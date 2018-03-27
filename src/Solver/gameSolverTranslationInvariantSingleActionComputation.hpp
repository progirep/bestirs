#ifndef __GAME_SOLVER_TRANSLATION_INVASTIANT_SINGLE_ACTION_COMPUTATION_HPP
#define __GAME_SOLVER_TRANSLATION_INVASTIANT_SINGLE_ACTION_COMPUTATION_HPP

#include "gameSolver.hpp"

template<int nofStateDimensions, int nofTranslationInvariantStateDimensions, bool andAbstractGrouping> class GameSolverTranslationInvariantSingleActionComputation : public GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions> {
protected:
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::mgr;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::bfVariables;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::bfVariableNames;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::transitionFunction;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::sizeWorkspace;
    std::vector<BF> bfActionVars;

    // Connecting the workspace with BDDs
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::bfStateVarsPre;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::bfStateVarsPost;

    // Function
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::encodeValueinBFs;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::enforcePreselectedVariableOrderIfGiven;

public:
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::drawXYStates;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::getSizeWorkspace;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::solveGenBuchiGame;

private:
    // BF/BDD Stuff
    BFVarVector allPreNontranslationInvariantVarVector;
    BFVarVector allPostNontranslationInvariantVarVector;
    BFVarCube allPostNontranslationInvariantVarCube;
    BFVarCube allActionCube;

    // All actions, given as collections of pairs of Displacement and BF over PreOther/PostOther vars
    // The two BFs are the pre/post combination and the set of non-transition-variant pre states from which this displacement
    // cannot occur.
    std::vector<std::tuple<std::array<int,nofTranslationInvariantStateDimensions>,BF> > globalActions;
    BF anyAction;

    // Substitution vectors
    std::map<std::array<int,nofTranslationInvariantStateDimensions>,std::vector<std::pair<BF,BF> > > substitutionVectorsTranslationInvariance;

    // For the EnfPre operator, we need to be able to restrict the positions of
    std::array<std::map<int,BF>,nofTranslationInvariantStateDimensions> positionRestrictionBFsForEnfPre;

public:
    GameSolverTranslationInvariantSingleActionComputation(TransitionFunction<nofStateDimensions,nofTranslationInvariantStateDimensions> &_transitionFunction, SettingReader &settingReader, bool dynamicReodering, std::string &imposedVariableOrdering);
    BF enfPre(BF targetStatesPre);
    unsigned int getActionToReachState(BF targetStatesPre, BF postStates);


    // Debugging
    void basicEnfPreTest();
    void drawSomeDebuggingInformation();
    void drawXYTransitions(std::vector<std::tuple<std::array<int,nofTranslationInvariantStateDimensions>,BF,BF> > &action, int sizeX, int sizeY, int middleX, int middleY);

};



/**
 * @brief Constructor -- Allocates all BDD variables and builds the abstraction transition BFs
 * @param _transitionFunction The transition function to be used.
 */
template<int nofStateDimensions, int nofTranslationInvariantStateDimensions, bool andAbstractGrouping>
GameSolverTranslationInvariantSingleActionComputation<nofStateDimensions,nofTranslationInvariantStateDimensions,andAbstractGrouping>::
GameSolverTranslationInvariantSingleActionComputation
(TransitionFunction<nofStateDimensions,nofTranslationInvariantStateDimensions> &_transitionFunction, SettingReader &settingReader, bool dynamicReodering, std::string &imposedVariableOrdering) : GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>(_transitionFunction, settingReader, dynamicReodering, imposedVariableOrdering) {

    // Allocate the needed BDDs -- State Bits
    std::vector<BF> allPreNontranslationInvariantVars;
    std::vector<BF> allPostNontranslationInvariantVars;

    for (int i=0; i<nofStateDimensions; i++) {
        int minValue = transitionFunction.getMinValues()[i];
        int maxValue = transitionFunction.getMaxValues()[i];
        int nofDifferentValues = maxValue-minValue;
        if (i>=nofTranslationInvariantStateDimensions) {
            sizeWorkspace[i] = nofDifferentValues+1;
        }
        int nofBits = 0;
        while ((1<<nofBits)<sizeWorkspace[i]) nofBits++;
        std::cerr << "Nof state bits for dimension " << i << ": " << nofBits << "\n";
        std::vector<BF> theseStateVarsPre;
        std::vector<BF> theseStateVarsPost;
        for (int j=0; j<nofBits; j++) {
            std::ostringstream varName;
            varName << transitionFunction.getStateDimensionName(i);
            varName << j;
            bfVariables.push_back(mgr.newVariable());
            bfVariableNames.push_back(varName.str());
            theseStateVarsPre.push_back(bfVariables.back());
            if (i>=nofTranslationInvariantStateDimensions) {
                allPreNontranslationInvariantVars.push_back(bfVariables.back());
                bfVariables.push_back(mgr.newVariable());
                bfVariableNames.push_back(varName.str()+"'");
                theseStateVarsPost.push_back(bfVariables.back());
                allPostNontranslationInvariantVars.push_back(bfVariables.back());
            }
        }
        bfStateVarsPre[i] = theseStateVarsPre;
        bfStateVarsPost[i] = theseStateVarsPost;
    }

    allPreNontranslationInvariantVarVector = mgr.computeVarVector(allPreNontranslationInvariantVars);
    allPostNontranslationInvariantVarVector = mgr.computeVarVector(allPostNontranslationInvariantVars);
    allPostNontranslationInvariantVarCube = mgr.computeCube(allPostNontranslationInvariantVars);

    enforcePreselectedVariableOrderIfGiven();

    // Group actions according to transition target
    std::map<int,std::list<std::array<int,2*nofStateDimensions+1>>> groupedTransitions;
    for (const std::array<int,2*nofStateDimensions+1> &t : transitionFunction.getTransitions()) {
        int actionNum = t[nofStateDimensions];
        groupedTransitions[actionNum].push_back(t);
    }

    // Encode transitions    
    std::vector<std::vector<std::tuple<std::array<int,nofTranslationInvariantStateDimensions>,BF,BF> > > actions;

    for (auto it : groupedTransitions) {
        std::map<std::array<int,nofTranslationInvariantStateDimensions>,BF> actionsPreVersion;

        for (const std::array<int,2*nofStateDimensions+1> &t : it.second) {

            // Encode in BFs
            BF thisTransition = mgr.constantTrue();
            std::array<int,nofTranslationInvariantStateDimensions> postTranslationInvariant;

            for (int i=0; i<nofStateDimensions; i++) {
                int minValue = transitionFunction.getMinValues()[i];
                //int maxValue = transitionFunction.getMaxValues()[i];
                //int nofDifferentValues = maxValue-minValue;
                if (i<nofTranslationInvariantStateDimensions) {
                    if (t[i]!=0) throw "Error in Transitions function -- first few elements are not always 0.";
                    postTranslationInvariant[i] = t[i+nofStateDimensions+1];
                } else {
                    thisTransition &= encodeValueinBFs(bfStateVarsPre[i],minValue,t[i]);
                    thisTransition &= encodeValueinBFs(bfStateVarsPost[i],minValue,t[nofStateDimensions+1+i]);
                }
            }

            if (actionsPreVersion.count(postTranslationInvariant)==0) {
                actionsPreVersion[postTranslationInvariant] = thisTransition;
            } else {
                actionsPreVersion[postTranslationInvariant] |= thisTransition;
            }
        }

        // Compute for which non-translation-invariant state this action is actually available.
        BF forWhichStatesIsThisActionAvailable = mgr.constantFalse();
        for (auto it : actionsPreVersion) {
            forWhichStatesIsThisActionAvailable |= it.second;
        }
        forWhichStatesIsThisActionAvailable = forWhichStatesIsThisActionAvailable.ExistAbstract(allPostNontranslationInvariantVarCube);

        std::vector<std::tuple<std::array<int,nofTranslationInvariantStateDimensions>,BF,BF> > asVec;
        asVec.resize(actionsPreVersion.size());
        typename std::map<std::array<int,nofTranslationInvariantStateDimensions>,BF>::iterator it2 = actionsPreVersion.begin();
        for (unsigned int i=0; i<actionsPreVersion.size(); i++) {
            asVec[i] = std::tuple<std::array<int,nofTranslationInvariantStateDimensions>,BF,BF>(it2->first,it2->second,forWhichStatesIsThisActionAvailable);
            it2++;
        }
        actions.push_back(asVec);
    }

    // Compute SubstitutionVectors
    for (auto it : actions) {
        for (auto it2 : it) {
            std::array<int,nofTranslationInvariantStateDimensions> const &relativeDisplacement = std::get<0>(it2);

            if (substitutionVectorsTranslationInvariance.count(relativeDisplacement)==0) {
                std::vector<std::pair<BF,BF> > theseSubstitutions;
                for (unsigned int dim=0; dim<nofTranslationInvariantStateDimensions; dim++) {

                    int localRelativeDisplacement = relativeDisplacement[dim];
                    for (unsigned int bitNo=0; bitNo < bfStateVarsPre[dim].size(); bitNo++) {
                        BF allCases = mgr.constantFalse();
                        for (int val=0; val<sizeWorkspace[dim]; val++) {
                            int relative = val+localRelativeDisplacement;
                            /*if ((relative>=0) && (relative<sizeWorkspace[dim])) */ {
                                if (relative & (1<<bitNo)) {
                                    BF thisCase = mgr.constantTrue();
                                    for (unsigned int bitNo2=0; bitNo2 < bfStateVarsPre[dim].size(); bitNo2++) {
                                        if (val & (1<<bitNo2)) {
                                            thisCase &= bfStateVarsPre[dim][bitNo2];
                                        } else {
                                            thisCase &= !bfStateVarsPre[dim][bitNo2];
                                        }
                                    }
                                    allCases |= thisCase;
                                }
                            }
                        }
                        if (allCases.isFalse()) throw "Error: Substitution is false.";
                        theseSubstitutions.push_back(std::pair<BF,BF>(bfStateVarsPre[dim][bitNo],allCases));
                    }
                }
                substitutionVectorsTranslationInvariance[relativeDisplacement] = theseSubstitutions;
            }
        }
    }

    // Compute "positionRestrictionBFsForEnfPre" -- Really unoptimal way to compute this,
    // but this is not the bottleneck.
    for (unsigned int dim=0; dim<nofTranslationInvariantStateDimensions; dim++) {

        // Get min/max values for displacement
        int minDisplacement = 0;
        int maxDisplacement = 0;
        for (auto &it : actions) {
            for (auto &action : it) {
                if (std::get<0>(action)[dim]<minDisplacement) minDisplacement = std::get<0>(action)[dim];
                if (std::get<0>(action)[dim]>maxDisplacement) maxDisplacement = std::get<0>(action)[dim];
            }
        }
        for (int relativeDisplacement=minDisplacement; relativeDisplacement<=maxDisplacement; relativeDisplacement++) {
            BF allCases = mgr.constantFalse();
            for (int val=0; val<sizeWorkspace[dim]; val++) {
                int relative = val+relativeDisplacement;
                if ((relative>=0) && (relative<sizeWorkspace[dim])) {
                    BF thisCase = mgr.constantTrue();
                    for (unsigned int bitNo2=0; bitNo2 < bfStateVarsPre[dim].size(); bitNo2++) {
                        if (relative & (1<<bitNo2)) {
                            thisCase &= bfStateVarsPre[dim][bitNo2];
                        } else {
                            thisCase &= !bfStateVarsPre[dim][bitNo2];
                        }
                    }
                    allCases |= thisCase;
                }
            }
            positionRestrictionBFsForEnfPre[dim][relativeDisplacement] = allCases;

            /*std::ostringstream fn;
            fn << "/tmp/positionRestrictionBFsFor" << dim << "-" << relativeDisplacement << ".dot";
            BF_newDumpDot(*this,allCases,NULL,fn.str());*/
        }
    }

    // Merge actions
    unsigned int nofActionVars = 0;
    while ((1UL<<nofActionVars)<actions.size()) {
        nofActionVars++;
    }
    for (unsigned int i=0;i<nofActionVars;i++) {
        bfActionVars.push_back(mgr.newVariable());
        bfVariables.push_back(bfActionVars.back());
        bfVariableNames.push_back("actionVar");
    }
    allActionCube = mgr.computeCube(bfActionVars);

    enforcePreselectedVariableOrderIfGiven();

    anyAction = mgr.constantFalse();
    BF theseAllowed = mgr.constantTrue();
    for (auto &allDisplacements : substitutionVectorsTranslationInvariance) {
        BF thesePost = mgr.constantTrue();
        for (unsigned int i=0;i<actions.size();i++) {
            BF encode = encodeValueinBFs(bfActionVars,0,i);
            for (auto &tuple : actions[i]) {
                if (std::get<0>(tuple)==allDisplacements.first) {
                    thesePost &= !encode | !std::get<1>(tuple);
                    theseAllowed &= !encode | std::get<2>(tuple);
                    anyAction |= encode;
                }
            }
        }
        //BF_newDumpDot(*this,theseAllowed,NULL,"/tmp/theseAllowed.dot");
        globalActions.push_back(std::tuple<std::array<int,nofTranslationInvariantStateDimensions>,BF>(allDisplacements.first,thesePost));
    }
    // std::map<std::array<int,nofTranslationInvariantStateDimensions>,std::vector<std::pair<BF,BF> > > substitutionVectorsTranslationInvariance;
    anyAction &= theseAllowed;


}


template<int nofStateDimensions, int nofTranslationInvariantStateDimensions, bool andAbstractGrouping>
BF GameSolverTranslationInvariantSingleActionComputation<nofStateDimensions,nofTranslationInvariantStateDimensions,andAbstractGrouping>::enfPre(BF targetStates) {

    if (andAbstractGrouping) {

        BF targetStatesPost = targetStates.SwapVariables(allPreNontranslationInvariantVarVector,allPostNontranslationInvariantVarVector);
        std::vector<BF> theseReachable;
        for (auto &pair : globalActions) {

            BF filteredPostStates = targetStatesPost;
            for (unsigned int i=0; i<nofTranslationInvariantStateDimensions; i++) {
                filteredPostStates &= positionRestrictionBFsForEnfPre[i][std::get<0>(pair)[i]];
            }

            auto &relative = std::get<0>(pair);
            /*std::ostringstream fn;
            fn << "/tmp/action" << relative[0] << "-" << relative[1];
            BF_newDumpDot(*this,std::get<2>(pair),NULL,fn.str()+"-or.dot");
            BF_newDumpDot(*this,std::get<1>(pair),NULL,fn.str()+"-base.dot");
            BF_newDumpDot(*this,filteredPostStates,NULL,"/tmp/filtered.dot");*/

            // Old:
            //BF these = filteredPostStates.compose(substitutionVectorsTranslationInvariance[relative]) | std::get<1>(pair);
            //theseReachable &= these.UnivAbstract(allPostNontranslationInvariantVarCube);

            // New:
            BF these = filteredPostStates.compose(substitutionVectorsTranslationInvariance[relative]);
            theseReachable.push_back(! (!these).AndAbstract(!std::get<1>(pair),allPostNontranslationInvariantVarCube));

        }

        //BF_newDumpDot(*this,theseReachable.ExistAbstract(allPostNontranslationInvariantVarCube),NULL,"/tmp/allreachable.dot");
        //throw 3;
        theseReachable.push_back(anyAction);

        return mgr.multiAnd(theseReachable).ExistAbstract(allActionCube);

    } else {

        BF targetStatesPost = targetStates.SwapVariables(allPreNontranslationInvariantVarVector,allPostNontranslationInvariantVarVector);
        BF theseReachable = mgr.constantTrue();
        for (auto &pair : globalActions) {

            BF filteredPostStates = targetStatesPost;
            for (unsigned int i=0; i<nofTranslationInvariantStateDimensions; i++) {
                filteredPostStates &= positionRestrictionBFsForEnfPre[i][std::get<0>(pair)[i]];
            }

            auto &relative = std::get<0>(pair);
            /*std::ostringstream fn;
            fn << "/tmp/action" << relative[0] << "-" << relative[1];
            BF_newDumpDot(*this,std::get<2>(pair),NULL,fn.str()+"-or.dot");
            BF_newDumpDot(*this,std::get<1>(pair),NULL,fn.str()+"-base.dot");
            BF_newDumpDot(*this,filteredPostStates,NULL,"/tmp/filtered.dot");*/

            // Old:
            //BF these = filteredPostStates.compose(substitutionVectorsTranslationInvariance[relative]) | std::get<1>(pair);
            //theseReachable &= these.UnivAbstract(allPostNontranslationInvariantVarCube);

            // New:
            BF these = filteredPostStates.compose(substitutionVectorsTranslationInvariance[relative]);
            theseReachable &= ! (!these).AndAbstract(!std::get<1>(pair),allPostNontranslationInvariantVarCube);

        }

        //BF_newDumpDot(*this,theseReachable.ExistAbstract(allPostNontranslationInvariantVarCube),NULL,"/tmp/allreachable.dot");
        //throw 3;

        return (theseReachable & anyAction).ExistAbstract(allActionCube);

    }

}


template<int nofStateDimensions, int nofTranslationInvariantStateDimensions, bool andAbstractGrouping>
unsigned int GameSolverTranslationInvariantSingleActionComputation<nofStateDimensions,nofTranslationInvariantStateDimensions,andAbstractGrouping>::getActionToReachState(BF targetStates, BF stateFrom) {
    (void)targetStates;
    (void)stateFrom;
    throw "Unsupported (GameSolverTranslationInvariantSingleActionComputation).";
}






//===========================================================
// Debugging functions
//===========================================================













template<int nofStateDimensions, int nofTranslationInvariantStateDimensions, bool andAbstractGrouping>
void GameSolverTranslationInvariantSingleActionComputation<nofStateDimensions,nofTranslationInvariantStateDimensions,andAbstractGrouping>::basicEnfPreTest() {

    // Requires X/Y workspace
    static_assert(nofTranslationInvariantStateDimensions==2,"basicEnfPreTest requires 2 dimensional transition invariant workspace");

    constexpr int sizeX = 17;
    constexpr int sizeY = 17;
    constexpr int middleX = (sizeX-1)/2;
    constexpr int middleY = (sizeY-1)/2;


    // Encode Target of 4x4 states in the middle of the workspace (x/y)
    BF targetStates = mgr.constantFalse();
    for (int x=0; x<4; x++) {
        for (int y=0; y<4; y++) {
            targetStates |= encodeValueinBFs(bfStateVarsPre[0],0,x+middleX) & encodeValueinBFs(bfStateVarsPre[1],0,y+middleY);
        }
    }

    // Overwrite target
    targetStates = (!bfStateVarsPre[0].back());

    BF_newDumpDot(*this,targetStates,NULL,"/tmp/targetStates.dot");

    std::cerr << "Target:\n";
    drawXYStates(targetStates,40,17,0,0);

    BF reach = enfPre(targetStates);
    std::cerr << "Pre:\n";
    drawXYStates(reach,40,17,0,0);
    BF_newDumpDot(*this,reach,NULL,"/tmp/pre.dot");


    // Compose test
    /*std::vector<std::pair<BF,BF> > composeTextVector;
    composeTextVector.push_back(std::pair<BF,BF>(bfStateVarsPre[0].back(),bfStateVarsPre[1].back() & bfStateVarsPre[0].back()));
    BF_newDumpDot(*this,targetStates.compose(composeTextVector),NULL,"/tmp/compose.dot");*/
}







#endif
