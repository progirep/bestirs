#ifndef __GAME_SOLVER_BIG_BDD_MULTI_ACTION_HPP
#define __GAME_SOLVER_BIG_BDD_MULTI_ACTION_HPP

#include "gameSolver.hpp"
#include <limits>

template<int nofStateDimensions, int nofTranslationInvariantStateDimensions, bool andAbstractGrouping> class GameSolverBigBDDMultiActionComputation : public GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions> {
protected:
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::mgr;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::bfVariables;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::bfVariableNames;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::bfIsPreStateVar;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::transitionFunction;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::sizeWorkspace;

    // Connecting the workspace with BDDs
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::bfStateVarsPre;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::bfStateVarsPost;

    // Function
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::encodeValueinBFs;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::enforcePreselectedVariableOrderIfGiven;

public:
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::drawXYStates;
    using GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>::solveGenBuchiGame;


private:
    // BF/BDD Stuff
    BFVarVector allPreVarVector;
    BFVarVector allPostVarVector;
    BFVarCube allPostVarCube;

    // All actions, given as collections of pairs of Displacement and BF over PreOther/PostOther vars
    // The two BFs are the pre/post combination and the set of non-transition-variant pre states from which this displacement
    // cannot occur.
    std::vector<std::pair<BF,BF>> actions;

public:
    GameSolverBigBDDMultiActionComputation(TransitionFunction<nofStateDimensions,nofTranslationInvariantStateDimensions> &_transitionFunction, SettingReader &settingsReader, bool dynamicReodering, std::string &imposedVariableOrdering);
    BF enfPre(BF targetStatesPre);
    unsigned int getActionToReachState(BF targetStates, BF stateFrom) { (void)targetStates; (void)stateFrom; throw "Unimplemented."; }
};



/**
 * @brief Constructor -- Allocates all BDD variables and builds the abstraction transition BFs
 * @param _transitionFunction The transition function to be used.
 */
template<int nofStateDimensions, int nofTranslationInvariantStateDimensions, bool andAbstractGrouping>
GameSolverBigBDDMultiActionComputation<nofStateDimensions,nofTranslationInvariantStateDimensions,andAbstractGrouping>::
GameSolverBigBDDMultiActionComputation
(TransitionFunction<nofStateDimensions,nofTranslationInvariantStateDimensions> &_transitionFunction, SettingReader &settingReader, bool dynamicReodering, std::string &imposedVariableOrdering) : GameSolver<nofStateDimensions,nofTranslationInvariantStateDimensions>(_transitionFunction,settingReader,dynamicReodering,imposedVariableOrdering) {

    // Allocate the needed BDDs -- State Bits
    std::vector<BF> allPreVars;
    std::vector<BF> allPostVars;

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
            bfIsPreStateVar.push_back(true);
            allPreVars.push_back(bfVariables.back());

            bfVariables.push_back(mgr.newVariable());
            bfVariableNames.push_back(varName.str()+"'");
            theseStateVarsPost.push_back(bfVariables.back());
            bfIsPreStateVar.push_back(false);
            allPostVars.push_back(bfVariables.back());
        }
        bfStateVarsPre[i] = theseStateVarsPre;
        bfStateVarsPost[i] = theseStateVarsPost;
    }

    allPreVarVector = mgr.computeVarVector(allPreVars);
    allPostVarVector = mgr.computeVarVector(allPostVars);
    allPostVarCube = mgr.computeCube(allPostVars);

    enforcePreselectedVariableOrderIfGiven();

    // Group actions according to transition target
    std::map<int,std::list<std::array<int,2*nofStateDimensions+1>>> groupedTransitions;
    for (const std::array<int,2*nofStateDimensions+1> &t : transitionFunction.getTransitions()) {
        int actionNum = t[nofStateDimensions];
        groupedTransitions[actionNum].push_back(t);
    }

    // Encode transitions
    for (auto it : groupedTransitions) {

        std::map<std::array<int,nofStateDimensions-nofTranslationInvariantStateDimensions>,std::array<int,nofTranslationInvariantStateDimensions> > localMin;
        std::map<std::array<int,nofStateDimensions-nofTranslationInvariantStateDimensions>,std::array<int,nofTranslationInvariantStateDimensions> > localMax;

        BF transitions = mgr.constantFalse();
        BF allowedCases = mgr.constantFalse();

        for (const std::array<int,2*nofStateDimensions+1> &t : it.second) {

            BF thisTransition = mgr.constantTrue();
            BF thisAllowedCase = mgr.constantTrue();

            for (int i=0; i<nofStateDimensions; i++) {
                int minValue = transitionFunction.getMinValues()[i];
                if (i<nofTranslationInvariantStateDimensions) {
                    if (t[i]!=0) throw "Error in Transitions function -- first few elements are not always 0.";
                    int post = t[nofStateDimensions+1+i];

                    std::array<int,nofStateDimensions-nofTranslationInvariantStateDimensions> localNonTranslationInvariant;
                    std::copy(t.data() + nofTranslationInvariantStateDimensions, t.data() + nofStateDimensions, localNonTranslationInvariant.data());
                    auto it = localMin.find(localNonTranslationInvariant);
                    if (it==localMin.end()) {
                        std::array<int,nofTranslationInvariantStateDimensions> localMinElement;
                        std::array<int,nofTranslationInvariantStateDimensions> localMaxElement;
                        for (unsigned int j=0;j<nofTranslationInvariantStateDimensions;j++) {
                            localMinElement[j] = std::numeric_limits<int>::max();
                            localMaxElement[j] = std::numeric_limits<int>::min();
                        }
                        localMinElement[i] = post;
                        localMaxElement[i] = post;
                        localMin[localNonTranslationInvariant] = localMinElement;
                        localMax[localNonTranslationInvariant] = localMaxElement;
                    } else {
                        localMin[localNonTranslationInvariant][i] = std::min(localMin[localNonTranslationInvariant][i],post);
                        localMax[localNonTranslationInvariant][i] = std::max(localMax[localNonTranslationInvariant][i],post);
                    }
                    // Encode from the center
                    thisTransition &= encodeValueinBFs(bfStateVarsPost[i],0,sizeWorkspace[i]/2+t[nofStateDimensions+1+i]);
                } else {
                    BF thisEncodingPre = encodeValueinBFs(bfStateVarsPre[i],minValue,t[i]);
                    thisTransition &= thisEncodingPre;
                    thisAllowedCase &= thisEncodingPre;
                    thisTransition &= encodeValueinBFs(bfStateVarsPost[i],minValue,t[nofStateDimensions+1+i]);
                }
            }

            transitions |= thisTransition;
            allowedCases |= thisAllowedCase;
        }

        // Copy the transitions for every dimension
        std::vector<std::vector<BF> > encodedPostValues;
        for (unsigned int dim=0;dim<nofTranslationInvariantStateDimensions;dim++) {
            std::vector<BF> valueBFs;
            for (int value = 0;value < sizeWorkspace[dim];value++) {
                BF thisCase = mgr.constantTrue();
                for (unsigned int bitNo2=0; bitNo2 < bfStateVarsPost[dim].size(); bitNo2++) {
                    if (value & (1<<bitNo2)) {
                        thisCase &= bfStateVarsPost[dim][bitNo2];
                    } else {
                        thisCase &= !bfStateVarsPost[dim][bitNo2];
                    }
                }
                valueBFs.push_back(thisCase);
                //BF_newDumpDot(*this,thisCase,NULL,"/tmp/valueBF.dot");
                //if (value==3) throw 3;
            }
            encodedPostValues.push_back(valueBFs);
        }

        // --> actual copy
        for (unsigned int dim=0;dim<nofTranslationInvariantStateDimensions;dim++) {
            BF localTransitions = transitions;
            BF localAllowedCases = allowedCases;
            transitions = mgr.constantFalse();
            allowedCases = mgr.constantFalse();
            for (int value = 0;value < sizeWorkspace[dim];value++) {
                for (auto key = localMin.begin();key!=localMin.end();key++) {
                    if ((value>=-1*key->second[dim]) && (value<(sizeWorkspace[dim]-localMax[key->first][dim]))) {
                        BF thisCase = mgr.constantTrue();
                        for (unsigned int i=0;i<nofStateDimensions-nofTranslationInvariantStateDimensions;i++) {
                            thisCase &= encodeValueinBFs(bfStateVarsPre[i+nofTranslationInvariantStateDimensions],transitionFunction.getMinValues()[nofTranslationInvariantStateDimensions+i],key->first[i]);
                        }
                        thisCase &= encodeValueinBFs(bfStateVarsPre[dim],0,value);
                        allowedCases |= localAllowedCases & thisCase;
                        BF these = localTransitions & thisCase;

                        // Substitute post
                        int relativeBase = sizeWorkspace[dim]/2-value;

                        std::vector<std::pair<BF,BF> > theseSubstitutions;

                        for (unsigned int bitNo=0; bitNo < bfStateVarsPre[dim].size(); bitNo++) {
                            BF allCases = mgr.constantFalse();
                            for (int val=0; val<sizeWorkspace[dim]; val++) {
                                int relative = val+relativeBase;
                                /*if ((relative>=0) && (relative<sizeWorkspace[dim])) */ {
                                    if (relative & (1<<bitNo)) {
                                        allCases |= encodedPostValues[dim][val];

                                        /*
                                        BF thisCase = mgr.constantTrue();
                                        for (unsigned int bitNo2=0; bitNo2 < bfStateVarsPost[dim].size(); bitNo2++) {
                                            if (val & (1<<bitNo2)) {
                                                thisCase &= bfStateVarsPost[dim][bitNo2];
                                            } else {
                                                thisCase &= !bfStateVarsPost[dim][bitNo2];
                                            }
                                        }
                                        allCases |= thisCase;*/
                                    }
                                }
                            }
                            if (allCases.isFalse()) throw "Error: Substitution is false.";
                            theseSubstitutions.push_back(std::pair<BF,BF>(bfStateVarsPost[dim][bitNo],allCases));
                        }
                        transitions |= these.compose(theseSubstitutions);
                    }
                }
            }
        }

        actions.push_back(std::pair<BF,BF>(transitions,allowedCases));
        //BF_newDumpDot(*this,transitions,NULL,"/tmp/transitions.dot");
        //BF_newDumpDot(*this,allowedCases,NULL,"/tmp/allowedCases.dot");

    }

}


template<int nofStateDimensions, int nofTranslationInvariantStateDimensions, bool andAbstractGrouping>
BF GameSolverBigBDDMultiActionComputation<nofStateDimensions,nofTranslationInvariantStateDimensions,andAbstractGrouping>::enfPre(BF targetStates) {

    if (andAbstractGrouping) {
        BF targetStatesPost = targetStates.SwapVariables(allPreVarVector,allPostVarVector);
        BF allReachable = mgr.constantFalse();
        for (auto &action : actions) {
            BF these = (targetStatesPost | !action.first).UnivAbstract(allPostVarCube) & action.second;
            allReachable |= these;
        }
        return allReachable;
    } else {
        BF targetStatesPost = targetStates.SwapVariables(allPreVarVector,allPostVarVector);
        std::vector<BF> parts;
        for (auto &action : actions) {
            BF these = (targetStatesPost | !action.first).UnivAbstract(allPostVarCube) & action.second;
            parts.push_back(!these);
        }
        return !(mgr.multiAnd(parts));
    }

}






//===========================================================
// Debugging functions
//===========================================================


















#endif
