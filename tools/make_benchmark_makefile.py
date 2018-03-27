#!/usr/bin/env python2
# Benchmark Makefile generator
#
import sys, os, glob, random, itertools, config

# SETTINGS
SOLVER_MODES = [(b,a,e) for (a,b,c,d,e) in config.CONFIG]

# Find all benchmarks
allBenchmarkFiles = glob.glob("*Setting.txt")
print allBenchmarkFiles

# Get relative path
basePath = sys.argv[0]
while basePath[-1]!="/":
    basePath=basePath[0:len(basePath)-1]
basePath = basePath = "../src/Solver/solver "

# Extensions
extensionsBenchmarkResults = [c for (a,c,e) in SOLVER_MODES]
commands = [a for (a,c,e) in SOLVER_MODES]

# Sorting helper function
def sorter(lineA,lineB):
    partsA = lineA.split("-")
    partsB = lineB.split("-")
    if len(partsA)!=len(partsB):
        return cmp(len(partsA),len(partsB))
    for i in range(0,len(partsA)):
        try:
            a = int(partsA[i])
            b = int(partsB[i])
            if a<b:
                return -1
            if a>b:
                return 1
        except ValueError:
            pass
    return cmp(lineA,lineB)
    
    
def getReorderingFilenameFromBestOrderedFilename(filename):
    return filename[0:filename.rfind("b")]+"r"+filename[filename.rfind("b")+1:]
        

with open("Makefile","w") as makefile:
    print >>makefile,"default :",
    dependencies = []
    for a in allBenchmarkFiles:
        for (e,bx,cx,dx,ex) in config.CONFIG:
            if not ex or glob.glob(getReorderingFilenameFromBestOrderedFilename(".res/"+a+e+".txt"))!=[]:
                dependencies.append(".res/"+a+e+".txt")
    dependencies = sorted(dependencies,cmp=sorter)

    for dependency in dependencies:
        print >>makefile,dependency,
    print >>makefile,"\n\t../tools/make_paper_cactusplot.py > cactus.tex\n\tpdflatex cactus.tex\n\tpdflatex cactus.tex"
    print >>makefile,"\n\techo \"Done!\"\n"

    for a in allBenchmarkFiles:
        for (param,ext,isGivenOrder) in SOLVER_MODES:

            if isGivenOrder:
                if glob.glob(getReorderingFilenameFromBestOrderedFilename(".res/"+a+ext+".txt"))!=[]:
                    with open(getReorderingFilenameFromBestOrderedFilename(".res/"+a+ext+".txt"),"r") as dataFile:
                        orderLine = None
                        gotTimeout = False
                        for line in dataFile.readlines():
                            if line.startswith("Final Variable order: "):
                                orderLine = line
                            elif line.startswith("TIMEOUT "):
                                gotTimeout = True
                        if gotTimeout:
                            with open(".res/"+a+ext+".txt","w") as outFile:
                                outFile.write("NA")
                        else:
                            if orderLine is None:
                                print >>sys.stderr,"Error reading order from "+getReorderingFilenameFromBestOrderedFilename(".res/"+a+ext+".txt")
                            else:
                                order = orderLine[22:].strip()
                                print >>makefile,".res/"+a+ext+".txt:"
                                print >>makefile,"\tmkdir -p .res"
                                print >>makefile,"\t../tools/timeout -m 3192000 -t 3600 "+basePath+param+" --variableOrder '"+order+"' "+a[0:len(a)-11]+".txt "+a+" > .res/"+a+ext+".txt 2>&1"
                                print >>makefile,"\t if grep -q \"^\+\-\-\" "+".res/"+a+ext+".txt ; then false; fi\n"
            else:
                print >>makefile,".res/"+a+ext+".txt:"
                print >>makefile,"\tmkdir -p .res"
                print >>makefile,"\t../tools/timeout -m 3192000 -t 3600 "+basePath+param+" "+a[0:len(a)-11]+".txt "+a+" > .res/"+a+ext+".txt 2>&1"
                print >>makefile,"\t if grep -q \"^\+\-\-\" "+".res/"+a+ext+".txt ; then false; fi\n"

    print >>makefile,"clean :"
    for a in allBenchmarkFiles:
        for (param,ext,isFixedOrder) in SOLVER_MODES:
            print >>makefile,"\trm -f .res/"+a+ext
    print >>makefile,""
