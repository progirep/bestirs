#ifndef __MULTI_AND_BDD_CUDD_H__
#define __MULTI_AND_BDD_CUDD_H__

#include <cstring>
#include <cuddInt.h>
#include <algorithm>
#include <unordered_map>


BFBdd BFBddManager::multiAnd(const std::vector<BFBdd> &parts) const {
    DdNode *soFar = Cudd_ReadOne(mgr);
    Cudd_Ref(soFar);
    for (std::vector<BFBdd>::const_iterator it = parts.begin(); it != parts.end(); it++) {
        DdNode *next = Cudd_bddAnd(mgr, soFar, it->node);
        //std::cerr << "(" << Cudd_DagSize(next) << ":" << Cudd_DagSize(it->node) << ")";
        Cudd_Ref(next);
        Cudd_RecursiveDeref(mgr, soFar);
        soFar = next;
    }
    //std::cerr << "(" << Cudd_DagSize(soFar) << ")\n";
    Cudd_Deref(soFar);
    return BFBdd(this, soFar);
}


inline DdNode *multiAndAbstractRecurse(DdManager *mgr, DdNode **subNodes, DdNode *cube, size_t nofNodes) {

    // Weed out constants
    DdNode *one = mgr->one;
    for (unsigned int i=0;i<nofNodes;i++) {
        DdNode *thisNode = subNodes[i];
        DdNode *G = Cudd_Regular(thisNode);
        if (G==one) {
            // Is this false?
            if (G!=thisNode) {
                cuddSatInc(one->ref);
                return thisNode;
            }
            // Else remove as this is a "TRUE"
            subNodes[i] = subNodes[nofNodes-1];
            nofNodes--;
            i--;
        }
    }

    // TRUE
    if (nofNodes==0) {
        cuddRef(one);
        return one;
    }

    // Sort
    std::sort(subNodes,subNodes+nofNodes);

    // Remove duplicates
    if (nofNodes>1) {

        //std::cerr << "Before:";
        //for (unsigned int i=0;i<nofNodes;i++) std::cerr << " " << subNodes[i];
        //std::cerr << "\n";

        unsigned int readPtr = 1;
        unsigned int writePtr = 1;
        while (readPtr<nofNodes) {
            size_t diff = (size_t)(subNodes[readPtr]) - (size_t)(subNodes[readPtr-1]);
            if (diff==0) {
                //std::cerr << "(*)";
                readPtr++;
            } else if ((diff==1) && (subNodes[readPtr] == Cudd_Not(subNodes[readPtr-1]))) { // A node and its negation
                cuddSatInc(one->ref);
                return Cudd_Not(one);
            } else {
                DdNode *pre = subNodes[readPtr++];
                subNodes[writePtr++] = pre;
            }
        }
        nofNodes = writePtr;

        //std::cerr << "-After:";
        //for (unsigned int i=0;i<nofNodes;i++) std::cerr << " " << subNodes[i];
        //std::cerr << "\n";
    }

    // Single node?
    if (nofNodes==1) {
        DdNode *r = cuddBddExistAbstractRecur(mgr,subNodes[0],cube);
        cuddRef(r);
        return r;
    }

    // Double-Node --> Use classical And
    if (nofNodes==2) {
        DdNode *r = cuddBddAndAbstractRecur(mgr,subNodes[0],subNodes[1],cube);
        cuddRef(r);
        return r;
    }


    // Ok, then it looks like we have to branch!
    int minIndex = CUDD_CONST_INDEX;
    unsigned int branchingVar = -1;
    for (unsigned int i=0;i<nofNodes;i++) {
        auto idx = Cudd_Regular(subNodes[i])->index;
        int thisOrder = mgr->perm[idx];
        if (thisOrder<minIndex) {
            minIndex = thisOrder;
            branchingVar = idx;
        }
    }

    std::cerr << "-After:";
    for (unsigned int i=0;i<nofNodes;i++) std::cerr << " " << subNodes[i];
    std::cerr << "\n";

    // Branch TRUE
    DdNode *subsT[nofNodes];
    DdNode *subsE[nofNodes];
    for (unsigned int i=0;i<nofNodes;i++) {
        if (Cudd_Regular(subNodes[i])->index==branchingVar) {
            if (Cudd_IsComplement(subNodes[i])) {
                DdNode *k = Cudd_Regular(subNodes[i]);
                subsT[i] = Cudd_Not(Cudd_T(k));
                subsE[i] = Cudd_Not(Cudd_E(k));
            } else {
                subsT[i] = Cudd_T(subNodes[i]);
                subsE[i] = Cudd_E(subNodes[i]);
            }
        } else {
            subsT[i] = subNodes[i];
            subsE[i] = subNodes[i];
        }
    }

    // Update cube?
    bool usedCube = false;
    while ((cube->index!=CUDD_CONST_INDEX) && (mgr->perm[cube->index]<minIndex)) {
        cube = Cudd_T(cube);
    }
    usedCube = cube->index==branchingVar;


    // Will be Ref'd by caches
    DdNode *t = multiAndAbstractRecurse(mgr,subsT,cube,nofNodes);
    if ((t==one) && (usedCube)) {
        return t;
    }
    DdNode *e = multiAndAbstractRecurse(mgr,subsE,cube,nofNodes);
    if (t==e) {
        cuddDeref(e);
        return t;
    } else {
        if (usedCube) {
            DdNode *r = Cudd_Not(cuddBddAndRecur(mgr,Cudd_Not(t),Cudd_Not(e)));
            cuddRef(r);
            cuddDeref(t);
            cuddDeref(e);
            return r;
        } else if (Cudd_IsComplement(t)) {
            DdNode *r = cuddUniqueInter(mgr,branchingVar,Cudd_Not(t),Cudd_Not(e));
            cuddRef(r);
            cuddDeref(t);
            cuddDeref(e);
            return Cudd_Not(r);
        } else {
            DdNode *r = cuddUniqueInter(mgr,branchingVar,t,e);
            cuddRef(r);
            cuddSatDec(t->ref);
            cuddDeref(e);
            return r;
        }
    }
}

#define USE_CLASSICAL_MULTI_AND_ABSTRACT

#ifdef USE_CLASSICAL_MULTI_AND_ABSTRACT
inline BFBdd BFBddManager::multiAndAbstract(const std::vector<BFBdd> &parts, BFBddVarCube &cube) const {

    BF res = constantTrue();
    unsigned int i=0;
    unsigned int max=parts.size()-1;
    while (i<max) {
        res &= parts[i];
        res &= parts[max];
        i++;
        max--;
    }
    if (i==max) res &= parts[i];
    //for (auto const &it : parts) res &= it;
    return res.ExistAbstract(cube);
}
#else

inline BFBdd BFBddManager::multiAndAbstract(const std::vector<BFBdd> &parts, BFBddVarCube &cube) const {

    // Deactivate variable reordering -- reenable it later (perhaps)
    int oldAutoDyn = mgr->autoDyn;
    mgr->autoDyn = 0;
    BF res;

    DdNode *result;
    {
        std::cerr << "New round!";
        DdNode *in[parts.size()+1];
        for (unsigned int i=0;i<parts.size();i++) in[i] = parts[i].node;
        in[parts.size()] = 0;
        result = multiAndAbstractRecurse(mgr,(DdNode**)in,cube.cube, parts.size());
        res = BFBdd(this,result);
        Cudd_RecursiveDeref(mgr,result);
    }
    //Cudd_Deref(result); // Because BFBdd Ref's this again.

    // Reenable automatic variable reordering (if it was switched on beforehand)
    mgr->autoDyn = oldAutoDyn;

    return res;
}
#endif


#endif
