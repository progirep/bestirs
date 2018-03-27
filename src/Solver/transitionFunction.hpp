#ifndef __TRANSITION_FUNCTION_HPP__
#define __TRANSITION_FUNCTION_HPP__

template<int nofStateDimensions, int nofTranslationInvariantStateDimensions> class TransitionFunction {
public:
    //constexpr static int nofStateDimensions = 3;
    //constexpr static int nofTranslationInvariantStateDimensions = 2;
private:
    std::array<int,nofStateDimensions> minValues;
    std::array<int,nofStateDimensions> maxValues;
    std::list<std::array<int,2*nofStateDimensions+1>> transitions;
public:
    TransitionFunction(const char *inputFilename);
    inline const std::array<int,nofStateDimensions> &getMinValues() {
        return minValues;
    }
    const std::array<int,nofStateDimensions> &getMaxValues() {
        return maxValues;
    }
    std::string getStateDimensionName(unsigned int dimension) const {
        if (dimension==0) return "x";
        if (dimension==1) return "y";
        if (dimension==2) return "dim2";
        if (dimension==3) return "dim3";
        throw "Unimplemented: getStateDimensionName dimension>2";
    }
    std::list<std::array<int,2*nofStateDimensions+1>> const &getTransitions() {
        return transitions;
    }
};

/**
 * @brief Constructor that reads a transition list from file.
 * @param inputFilename
 */
template<int nofStateDimensions, int nofTranslationInvariantStateDimensions> TransitionFunction<nofStateDimensions,nofTranslationInvariantStateDimensions>::TransitionFunction(const char *inputFilename) {
    std::ifstream inFile(inputFilename);
    if (inFile.fail()) throw "Error open transition list file.";
    std::string currentLine;
    for (int i=0; i<nofStateDimensions; i++) {
        minValues[i] = std::numeric_limits<int>::max();
        maxValues[i] = std::numeric_limits<int>::min();
    }
    while (std::getline(inFile,currentLine)) {
        if (currentLine.size()>0) {
            if (currentLine[0]!='c') {
                std::vector<std::string> parts = stringSplit(currentLine,',');
                if (parts.size()!=2*nofStateDimensions+1) {
                    throw std::string("Line has not the correct number of parts: '")+currentLine+"'";
                }
                transitions.push_back(std::array<int,2*nofStateDimensions+1>());
                for (int i=0; i<2*nofStateDimensions+1; i++) {
                    transitions.back()[i] = toInt(parts[i]);
                }
                for (int i=0; i<nofStateDimensions; i++) {
                    minValues[i] = std::min(minValues[i],std::min(transitions.back()[i],transitions.back()[i+1+nofStateDimensions]));
                    maxValues[i] = std::max(maxValues[i],std::max(transitions.back()[i],transitions.back()[i+1+nofStateDimensions]));
                }
            }
        }
    }
    if (inFile.bad()) throw "Error reading transition file.";
}

#endif // __TRANSITION_FUNCTION_HPP__
