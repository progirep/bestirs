#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <map>
#include <limits>
#include <BF.h>
#include "bddDump.h"
#include "tools.hpp"
#include "settingReader.hpp"
#include "transitionFunction.hpp"
#include "gameSolverTranslationInvariantMultiActionComputation.hpp"
#include "gameSolverBigBDDMultiActionComputation.hpp"
#include "gameSolverBigBDDSingleActionComputation.hpp"
#include "gameSolverTranslationInvariantSingleActionComputation.hpp"
#include "gameSolverTranslationInvariantSingleActionMultiComposeComputation.hpp"

typedef enum {
    ClassicalMonolithicSingleBDDEnfPreComputation,
    ClassicalMonolithicBDDPerActionEnfPreComputation,
    TranslationInvariantSingleActionComputation,
    TranslationInvariantMultiActionComputation,
    TranslationInvariantSingleActionMultiCompositionComputation,
} EnfPreAlgorithmVariant;

/**
 * Third level instantiator, Fixed is the algorithm variant and the dimension sizes,
 */
template<class algorithm, int nofDim, int nofTIDim> void instantiateAlgorithmC(const char *abstractionFilename, SettingReader &reader, bool interactive, bool dynamicReodering,std::string imposedVariableOrdering) {
    TransitionFunction<nofDim,nofTIDim> transitionFunction(abstractionFilename);
    algorithm solver(transitionFunction,reader,dynamicReodering,imposedVariableOrdering);
    //solver.drawSomeDebuggingInformation();
    //solver.basicEnfPreTest();
    std::string solverMode = reader.getStringValue("SolverMode");
    if (solverMode=="GeneralizedBuchi") {
        solver.solveGenBuchiGame(interactive);
    } else {
        throw "Error: Unknown SolverMode.";
    }

    /*if (dynamicReodering)*/ {
        std::cout << "\nFinal Variable order: ";
        Cudd_PrintGroupedOrder(solver.getMgr().getMgr(),"BDD",0);
    }

    //BF controllable = solver.computeExampleBuchiGame();
    //std::cerr << "Controllable states:\n";
    //solver.drawXYStates(controllable,solver.getSizeWorkspace()[0],solver.getSizeWorkspace()[1],0,0);
}


/**
 * Second level instantiator, Fixed is the algorithm variant
 */
template<int nofDim, int nofTIDim> void instantiateAlgorithmB(EnfPreAlgorithmVariant algorithm, const char *abstractionFilename, SettingReader &reader, bool interactive, bool dynamicsReordering, bool andAbstractGrouping, std::string imposedVariableOrdering) {
    if (andAbstractGrouping) {
        if (algorithm==TranslationInvariantMultiActionComputation) {
            instantiateAlgorithmC<GameSolverTranslationInvariantMultiActionComputation<nofDim,nofTIDim,true>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else if (algorithm==ClassicalMonolithicBDDPerActionEnfPreComputation) {
            instantiateAlgorithmC<GameSolverBigBDDMultiActionComputation<nofDim,nofTIDim,true>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else if (algorithm==ClassicalMonolithicSingleBDDEnfPreComputation) {
            instantiateAlgorithmC<GameSolverBigBDDSingleActionComputation<nofDim,nofTIDim>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else if (algorithm==TranslationInvariantSingleActionComputation) {
            instantiateAlgorithmC<GameSolverTranslationInvariantSingleActionComputation<nofDim,nofTIDim,true>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else if (algorithm==TranslationInvariantSingleActionMultiCompositionComputation) {
            instantiateAlgorithmC<GameSolverTranslationInvariantSingleActionMultiCompositionComputation<nofDim,nofTIDim,true>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else {
            throw "Unsupported algorithm";
        }
    } else {
        if (algorithm==TranslationInvariantMultiActionComputation) {
            instantiateAlgorithmC<GameSolverTranslationInvariantMultiActionComputation<nofDim,nofTIDim,false>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else if (algorithm==ClassicalMonolithicBDDPerActionEnfPreComputation) {
            instantiateAlgorithmC<GameSolverBigBDDMultiActionComputation<nofDim,nofTIDim,false>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else if (algorithm==ClassicalMonolithicSingleBDDEnfPreComputation) {
            instantiateAlgorithmC<GameSolverBigBDDSingleActionComputation<nofDim,nofTIDim>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else if (algorithm==TranslationInvariantSingleActionComputation) {
            instantiateAlgorithmC<GameSolverTranslationInvariantSingleActionComputation<nofDim,nofTIDim,false>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else if (algorithm==TranslationInvariantSingleActionMultiCompositionComputation) {
            instantiateAlgorithmC<GameSolverTranslationInvariantSingleActionMultiCompositionComputation<nofDim,nofTIDim,false>,nofDim,nofTIDim>(abstractionFilename,reader,interactive,dynamicsReordering,imposedVariableOrdering);
        } else {
            throw "Unsupported algorithm";
        }
    }
}


/**
 * Top level instantiator
 */
void instantiateAlgorithmA(EnfPreAlgorithmVariant translationInvariantMultiActionComputation, const char *filenameAbstraction, const char *filenameSetting, bool interactive, bool dynamicsReordering, bool andAbstractGrouping, std::string imposedVariableOrdering) {
    SettingReader reader(filenameSetting);
    int nofDims = reader.getValue("nofDimensions");
    int nofTranslationInvariantDims = reader.getValue("nofTranslationInvariantDimensions");

    if ((nofDims==3) && (nofTranslationInvariantDims==2)) {
        instantiateAlgorithmB<3,2>(translationInvariantMultiActionComputation,filenameAbstraction,reader,interactive,dynamicsReordering, andAbstractGrouping,imposedVariableOrdering);
    } else if ((nofDims==4) && (nofTranslationInvariantDims==2)) {
        instantiateAlgorithmB<4,2>(translationInvariantMultiActionComputation,filenameAbstraction,reader,interactive,dynamicsReordering, andAbstractGrouping,imposedVariableOrdering);
    } else if ((nofDims==2) && (nofTranslationInvariantDims==0)) {
        instantiateAlgorithmB<2,0>(translationInvariantMultiActionComputation,filenameAbstraction,reader,interactive,dynamicsReordering, andAbstractGrouping,imposedVariableOrdering);
    } else {
        throw "Error: Dimension number combination is currently not supported.";
    }
}


int main(int nofArgs, const char **args) {

    try {

        // Parse command line
        std::string inputFilenameAbstraction = "";
        std::string inputFilenameSetting = "";
        bool interactive = false;
        bool dynamicReordering = false;
        std::string variableOrdering = "";
        bool andAbstractGrouping = true;
        EnfPreAlgorithmVariant variant = TranslationInvariantMultiActionComputation;

        for (int i=1; i<nofArgs; i++) {
            if (std::string(args[i]).substr(0,1)=="-") {
                std::string param = std::string(args[i]);
                if (param=="--multiStepMultiAction") {
                    variant = TranslationInvariantMultiActionComputation;
                } else if (param=="--multiStepSingleAction") {
                    variant = TranslationInvariantSingleActionComputation;
                } else if (param=="--singleStepMultiAction") {
                    variant = ClassicalMonolithicBDDPerActionEnfPreComputation;
                } else if (param=="--singleStepSingleAction") {
                    variant = ClassicalMonolithicSingleBDDEnfPreComputation;
                } else if (param=="--multiStepSingleActionMultiCompose") {
                    variant = TranslationInvariantSingleActionMultiCompositionComputation;
                } else if (param=="--interactive") {
                    interactive = true;
                } else if (param=="--dynamicReordering") {
                    dynamicReordering = true;
                } else if (param=="--andAbstractGrouping") {
                    andAbstractGrouping = true;
                } else if (param=="--variableOrder") {
                    if (i==nofArgs-1) throw "Error: Need a variable order after '--variableOrder'";
                    variableOrdering = args[++i];
                } else {
                    std::cerr << "Error: Parameter '" << args[i] << "' not supported.\n";
                    std::cerr << "Options:\n";
                    std::cerr << "--multiStepMultiAction\n--multiStepSingleAction\n--singleStepMultiAction\n--singleStepSingleAction\n--multiStepSingleActionMultiCompose\n--interactive\n";
                    return 1;
                }
            } else {
                if (inputFilenameAbstraction=="") {
                    inputFilenameAbstraction = args[i];
                } else if (inputFilenameSetting=="") {
                    inputFilenameSetting = args[i];
                } else {
                    std::cerr << "Error: Too many file names given.\n";
                    return 1;
                }
            }
        }
        if (inputFilenameSetting=="") {
            std::cerr << "Error: Need at least two file names.\n";
            return 1;
        }

        // Start game
        instantiateAlgorithmA(variant,inputFilenameAbstraction.c_str(),inputFilenameSetting.c_str(),interactive,dynamicReordering,andAbstractGrouping,variableOrdering);

    } catch (const char *c) {
        std::cerr << c << std::endl;
        return 1;
    } catch (std::string c) {
        std::cerr << c << std::endl;
        return 1;
    } catch (BFDumpDotException e) {
        std::cerr << e.getMessage() << std::endl;
        return 1;
    }
}

