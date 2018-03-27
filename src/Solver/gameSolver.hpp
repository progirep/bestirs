#ifndef __GAME_SOLVER_HPP__
#define __GAME_SOLVER_HPP__

#include "settingReader.hpp"
#include <cassert>

template<int nofStateDimensions, int nofTranslationInvariantStateDimensions> class GameSolver : public VariableInfoContainer {
protected:

    // Settings
    SettingReader &settingReader;

    // BF/BDD stuff
    BFManager mgr;
    std::vector<BF> bfVariables;
    std::vector<std::string> bfVariableNames;
    std::vector<bool> bfIsPreStateVar;

    // Workspace information
    TransitionFunction<nofStateDimensions,nofTranslationInvariantStateDimensions> &transitionFunction;
    std::array<int,nofStateDimensions> sizeWorkspace;

    // Connecting the workspace with BDDs
    std::array<std::vector<BF>,nofStateDimensions> bfStateVarsPre;
    std::array<std::vector<BF>,nofStateDimensions> bfStateVarsPost;

    GameSolver(TransitionFunction<nofStateDimensions,nofTranslationInvariantStateDimensions> &_transitionFunction, SettingReader &_settingReader, bool dynamicReordering, std::string &imposedVariableOrdering);

    // Predefined variable order
    std::string predefinedVariableOrder;
    void enforcePreselectedVariableOrderIfGiven();

public:

    // Provided functions
    BF encodeValueinBFs(std::vector<BF> vars, int min, int value);
    BF computeControllableStates();
    virtual BF enfPre(BF targetStatesPre) = 0;
    virtual unsigned int getActionToReachState(BF targetStates, BF stateFrom) = 0;
    void drawXYStates(BF stateSet, int sizeX, int sizeY, int middleX, int middleY);
    BF computeExampleBuchiGame();
    virtual void solveGenBuchiGame(bool interactive);
    BF parseRegion(std::string regionDescription); // For the setting file.
    const BFManager &getMgr() const { return mgr; };

    // Accessor functions
    std::array<int,nofStateDimensions> const &getSizeWorkspace() const { return sizeWorkspace; }


    // Variable info container
    void getVariableTypes(std::vector<std::string> &types) const {
        types.push_back("all");
    }
    virtual void getVariableNumbersOfType(std::string type, std::vector<unsigned int> &nums) const {
        (void)type;
        for (unsigned int i=0; i<bfVariables.size(); i++) {
            nums.push_back(i);
        }
    }
    virtual BF getVariableBF(unsigned int number) const {
        return bfVariables[number];
    }
    virtual std::string getVariableName(unsigned int number) const {
        return bfVariableNames[number];
    }


};


template<int nofStateDimensions, int nofTranslationInvariantStateDimensions> void
GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::
enforcePreselectedVariableOrderIfGiven() {
    if (predefinedVariableOrder!="") {
        // Sanity parse
        if (predefinedVariableOrder.substr(0,1)!="(") throw "Illegal variable order string (reason 1)";
        if (predefinedVariableOrder.substr(predefinedVariableOrder.length()-1,predefinedVariableOrder.length())!=")") throw "Illegal variable order string (reason 2)";

        std::vector<std::string> parts = stringSplit(predefinedVariableOrder.substr(1,predefinedVariableOrder.length()-2),',');

        int varOrder[parts.size()];
        for (unsigned int i=0;i<parts.size();i++) {
            std::istringstream reader(parts[i]);
            int b;
            reader >> b;
            if (reader.fail()) throw "Error reading number from order.";
            varOrder[i] = b;
        }

        // Reduce varOrder by those entries that are larger than the current number of variables
        unsigned int reader = 0;
        unsigned int writer = 0;
        while (reader<parts.size()) {
            if (varOrder[reader]<static_cast<int>(bfVariables.size())) {
                varOrder[writer] = varOrder[reader];
                writer++;
            }
            reader++;
        }
        if (Cudd_ShuffleHeap(mgr.getMgr(),varOrder)!=1) throw "Variable ordering chage failed.";

    }

}

template<int nofStateDimensions, int nofTranslationInvariantStateDimensions>
GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::
GameSolver
(TransitionFunction<nofStateDimensions,nofTranslationInvariantStateDimensions> &_transitionFunction, SettingReader &_settingReader, bool dynamicReodering, std::string &imposedVariableOrdering) : settingReader(_settingReader), transitionFunction(_transitionFunction) {

    // Define workspace size
    for (unsigned int i=0; i<nofTranslationInvariantStateDimensions; i++) {
        sizeWorkspace[i] = settingReader.getValue("WorkspaceSizeDim"+toString(i));
    }

    // Disable automatic variable reordering
    mgr.setAutomaticOptimisation(dynamicReodering);

    // Impose a variable ordering (if given)
    predefinedVariableOrder = imposedVariableOrdering;


}

/**
 * @brief Encodes an integer value into the given BF variables
 * @param vars The BF variables to be used (in least significant bit first order)
 * @param min The minimal value of the dimension
 * @param max The maximal value of the dimension
 * @param value The value to be encoded
 * @return The final BF.
 */
template<int nofStateDimensions, int nofTranslationInvariantStateDimensions> BF GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::encodeValueinBFs(std::vector<BF> vars, int min, int value) {
    BF result = mgr.constantTrue();
    unsigned int toEncode = value - min;
    for (unsigned int i=0; i<vars.size(); i++) {
        if (toEncode & (1 << i)) {
            result &= vars[i];
        } else {
            result &= !vars[i];
        }
    }
    return result;
}



template<int nofStateDimensions, int nofTranslationInvariantStateDimensions>
BF GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::computeControllableStates() {

    BF currentlyControllable = mgr.constantTrue();
    BF lastControllable = mgr.constantFalse();
    while (currentlyControllable!=lastControllable) {
        lastControllable = currentlyControllable;
        currentlyControllable = enfPre(currentlyControllable);
        //std::cerr << "Prefix states states:\n";
        //drawXYStates(currentlyControllable,64,64,0,0);

    }
    return currentlyControllable;
}


template<int nofStateDimensions, int nofTranslationInvariantStateDimensions>
void GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::drawXYStates(BF stateSet, int sizeX, int sizeY, int middleX, int middleY) {

    // Make variable vector for all dimensions except for the first two.
    std::vector<BF> dimTwoPlusPreVars;
    for (unsigned int i=nofTranslationInvariantStateDimensions; i<nofStateDimensions; i++) {
        dimTwoPlusPreVars.insert(dimTwoPlusPreVars.end(),bfStateVarsPre[i].begin(),bfStateVarsPre[i].end());
    }

    std::cerr << "+";
    for (int i=0; i<sizeX; i++) std::cerr << "-";
    std::cerr << "+\n";
    for (int y=0; y<sizeY; y++) {
        std::cerr << "|";
        for (int x=0; x<sizeX; x++) {
            BF thisTarget = encodeValueinBFs(bfStateVarsPre[0],0,x-middleX) & encodeValueinBFs(bfStateVarsPre[1],0,y-middleY);
            if ((stateSet & thisTarget).isFalse()) {
                std::cerr << ".";
            } else {
                int count = (9*countAssignments(dimTwoPlusPreVars,0,stateSet & thisTarget)) / (1 << dimTwoPlusPreVars.size());
                std::cerr << count;
            }
        }
        std::cerr << "|\n";
    }
    std::cerr << "+";
    for (int i=0; i<sizeX; i++) std::cerr << "-";
    std::cerr << "+\n";
}




template<int nofStateDimensions, int nofTranslationInvariantStateDimensions>
BF GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::computeExampleBuchiGame() {

    BF goal = mgr.constantFalse();
    for (int x=sizeWorkspace[0]/4; x<sizeWorkspace[0]/2; x++) {
        for (int y=sizeWorkspace[1]/4; y<sizeWorkspace[1]/2; y++) {
            goal |= encodeValueinBFs(bfStateVarsPre[0],0,x) & encodeValueinBFs(bfStateVarsPre[1],0,y);
        }
    }
    //goal &= encodeValueinBFs(bfStateVarsPre[2],0,1);

    BF currentlyControllable = mgr.constantTrue();
    BF lastControllable = mgr.constantFalse();
    while (currentlyControllable!=lastControllable) {
        lastControllable = currentlyControllable;

        // Buchi
#ifdef DEBUGGING_2D_MAP
        std::cerr << "Local Goal:\n";
        drawXYStates(goal & currentlyControllable,sizeWorkspace[0],sizeWorkspace[1],0,0);
#endif
        BF localReach = enfPre(goal & currentlyControllable);
#ifdef DEBUGGING_2D_MAP
        std::cerr << "Initial Local Reach:\n";
        drawXYStates(localReach,sizeWorkspace[0],sizeWorkspace[1],0,0);
#endif

        BF oldReach = mgr.constantFalse();
        while (localReach!=oldReach) {
            oldReach = localReach;
            localReach |= enfPre(localReach);
#ifdef DEBUGGING_2D_MAP
            std::cerr << "Local Reach:\n";
            drawXYStates(localReach,sizeWorkspace[0],sizeWorkspace[1],0,0);
#endif
            //BF_newDumpDot(*this,localReach,NULL,"/tmp/intermLocalReach.dot");
            //throw 3;
        }
        currentlyControllable = localReach;
    }
    return currentlyControllable;
}


template<int nofStateDimensions, int nofTranslationInvariantStateDimensions>
BF GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::parseRegion(std::string regionDescription) {

    std::vector<std::string> parts = stringSplit(regionDescription,',');
    if (parts.size()!=nofStateDimensions) {
        std::ostringstream os;
        os << "Error while parsing region: '"+regionDescription+"' does not have as many comma-separated components as there are dimensions.";
        throw os.str();
    }
    BF all = mgr.constantTrue();
    for (unsigned int i=0;i<nofStateDimensions;i++) {
        std::string part = parts[i];
        if (part=="*") {
            // Any, so skip
        } else {
            size_t posMinus = part.find("-");
            if (posMinus==std::string::npos) {
                std::ostringstream os;
                os << "Error while parsing region: '"+regionDescription+"' has a non-star component without the '-' delimiter";
                throw os.str();
            }
            std::string part1 = part.substr(0,posMinus);
            std::string part2 = part.substr(posMinus+1,std::string::npos);
            int from = toInt(part1);
            int to = toInt(part2);
            BF cases = mgr.constantFalse();
            for (int j=from;j<=to;j++) {
                //std::cerr << "Parse"<<regionDescription<<"Dim"<<i<<"val"<<j<<std::endl;
                if (i<nofTranslationInvariantStateDimensions)
                    cases |= encodeValueinBFs(bfStateVarsPre[i],0,j);
                else
                    cases |= encodeValueinBFs(bfStateVarsPre[i],transitionFunction.getMinValues()[i],j);
            }
            all &= cases;
        }
    }
    //BF_newDumpDot(*this,all,NULL,std::string("/tmp/parse"+regionDescription+".dot"));
    return all;

}

template<int nofStateDimensions, int nofTranslationInvariantStateDimensions>
void GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::solveGenBuchiGame(bool interactive) {

    unsigned int nofGenBuchiGoals = safeDoubleToInt(settingReader.getValue("NofGoals"));
    unsigned int nofGenBuchiAvoidRegions = safeDoubleToInt(settingReader.getValue("NofBadRegions"));

    // Parse Buchi goals
    std::vector<BF> buchiGoals;
    for (unsigned int i=0;i<nofGenBuchiGoals;i++) {
        buchiGoals.push_back(parseRegion(settingReader.getStringValue("Goal"+toString(i))));
    }
    BF badRegions = mgr.constantFalse();
    for (unsigned int i=0;i<nofGenBuchiAvoidRegions;i++) {
        badRegions |= parseRegion(settingReader.getStringValue("BadRegion"+toString(i)));
    }

    BF currentlyControllable = !badRegions;
    BF lastControllable = mgr.constantFalse();

    // First, compute controllable part
    while (currentlyControllable!=lastControllable) {
        lastControllable = currentlyControllable;
        currentlyControllable &= enfPre(currentlyControllable);
#ifdef DEBUGGING_2D_MAP
            std::cerr << "Stable prefix set:\n";
            drawXYStates(currentlyControllable,sizeWorkspace[0],sizeWorkspace[1],0,0);
#endif
    }
    lastControllable = mgr.constantFalse();

    // Now do the Gen.Buchi iteration.
    std::vector<std::vector<BF> > prefixPoints;
    while (currentlyControllable!=lastControllable) {
        lastControllable = currentlyControllable;
        prefixPoints.clear();

        // Buchi
        for (auto &goal : buchiGoals) {
            if (interactive) prefixPoints.push_back({goal & currentlyControllable});

#ifdef DEBUGGING_2D_MAP
            std::cerr << "Local Goal:\n";
            drawXYStates(goal & currentlyControllable,sizeWorkspace[0],sizeWorkspace[1],0,0);
#endif
            BF localReach = enfPre(goal & currentlyControllable);
#ifdef DEBUGGING_2D_MAP
            std::cerr << "Initial Local Reach:\n";
            drawXYStates(localReach,sizeWorkspace[0],sizeWorkspace[1],0,0);
#endif

            BF oldReach = mgr.constantFalse();
            while (localReach!=oldReach) {

                if (interactive) {
                    prefixPoints.back().push_back(localReach);
                }

                oldReach = localReach;
                localReach |= enfPre(localReach) & !badRegions;
#ifdef DEBUGGING_2D_MAP
                std::cerr << "Local Reach:\n";
                drawXYStates(localReach,sizeWorkspace[0],sizeWorkspace[1],0,0);
#endif
            //BF_newDumpDot(*this,localReach,NULL,"/tmp/intermLocalReach.dot");
            //throw 3;
            }
            currentlyControllable &= localReach;
        }
        assert (((currentlyControllable & !lastControllable).isFalse()));

        //BF_newDumpDot(*this,lastControllable ^ currentlyControllable,NULL,"/tmp/new.dot");
    }

    /** Run interactive version */
    if (!interactive) {
        if (currentlyControllable.isFalse()) {
            std::cout << "System is uncontrollable from every state!" << std::endl;
        } else {
            std::cout << "System is controllable from some state!" << std::endl;
        }
    } else {
        if (currentlyControllable.isFalse()) {
            std::cerr << "System is uncontrollable from every state!" << std::endl;
            return;
        }
        unsigned int currentGoal = 0;

        // If there are no goals, make one to allow the simulator to execute.
        if (prefixPoints.size()==0) {
            prefixPoints.push_back({currentlyControllable});
            nofGenBuchiGoals = 1;
        }
        while (true) {
            BF currentState = mgr.constantTrue();
            std::array<int,nofStateDimensions> stateComponents;
            for (unsigned int i=0;i<nofStateDimensions;i++) {
                std::string nextData;
                if (!std::getline(std::cin,nextData)) throw "Failed to read next state component.";
                std::istringstream is(nextData);
                int value;
                is >> value;
                stateComponents[i] = value;
                if (is.bad()) throw "Parse error.";
                if (i<nofTranslationInvariantStateDimensions)
                    currentState &= encodeValueinBFs(bfStateVarsPre[i],0,value);
                else
                    currentState &= encodeValueinBFs(bfStateVarsPre[i],transitionFunction.getMinValues()[i],value);
            }

            if ((currentState & prefixPoints[currentGoal].back())==currentState) {

                // Search for first element containing the state
                unsigned int index = 0;
                while (!(currentState < prefixPoints[currentGoal][index])) index++;
                if (index==0) {
                    currentGoal = (currentGoal + 1) % nofGenBuchiGoals;
                    std::cout << "CS " << currentGoal << "\n";
                    index = 0;
                    while (!(currentState < prefixPoints[currentGoal][index])) {
                        index++;
                    }
                } else {
                    std::cout << "CG " << currentGoal << "\n";
                }

                int res;
                if (index>0)
                    res = getActionToReachState(prefixPoints[currentGoal][index-1],currentState);
                else
                    res = getActionToReachState(prefixPoints[currentGoal][index],currentState);
                std::cout << "Action " << res << "\n";

                // Execute action
                for (auto const &entry : transitionFunction.getTransitions()) {
                    bool applicable = (entry[nofStateDimensions]==res);
                    for (unsigned int i=nofTranslationInvariantStateDimensions;i<nofStateDimensions;i++) {
                        applicable &= (stateComponents[i]==entry[i]);
                    }
                    if (applicable) {
                        for (unsigned i=0;i<nofStateDimensions;i++) {
                             if (i>0) std::cout << ",";
                            if (i<nofTranslationInvariantStateDimensions) {
                                std::cout << entry[nofStateDimensions+1+i]+stateComponents[i];
                            } else {
                                std::cout << entry[nofStateDimensions+1+i];
                            }
                        }
                        std::cout << "\n";
                        //std::cout << "TRANS: " << entry[0] << "," << entry[1] << "," << entry[2] << "," << entry[3] << "," << entry[4] << "," << entry[5] << "," << entry[6] << "," << entry[7] << "\n";
                    }

                }
                std::cout << "DONE" << std::endl;
            } else {
                std::cout << "LOSINGSTATE" << std::endl;
            }
        }
    }
}

#endif
