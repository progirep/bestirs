#!/usr/bin/env python3
# Generate SAT instances for factoring integers
import sys, glob, os, math


config = [#('mma', '--multiStepMultiAction', 'Translation-Invariant, Multi-Action', 'color=yellow!70!black,thick', False), 
          #('mmra', '--multiStepMultiAction --dynamicReordering', 'Translation-Invariant, Multi-Action, Reordering', 'color=yellow!70!black', False), 
          #('mmba', '--multiStepMultiAction', 'Translation-Invariant, Multi-Action, Best Order', 'color=yellow!70!black,densely dotted', True), 
          #('sma', '--singleStepMultiAction', 'Classic EnfPre, Multi-Action', 'color=green!90!black,thick', False), 
          #('smra', '--singleStepMultiAction --dynamicReordering', 'Classic EnfPre, Multi-Action, Reordering', 'color=green!90!black', False), 
          #('smba', '--singleStepMultiAction', 'Classic EnfPre, Multi-Action, Best Ordering', 'color=green!90!black,densely dotted', True), 
          ('msa', '--multiStepSingleAction', 'Translation-Invariant', 'color=blue,thick', False), 
          ('msra', '--multiStepSingleAction --dynamicReordering', 'Translation-Invariant, Reordering', 'color=blue', False), 
          ('msba', '--multiStepSingleAction', 'Tranlation-Invariant, Best Order', 'color=blue,densely dotted', True), 
          ('ss', '--singleStepSingleAction', 'Classic EnfPre', 'color=red,thick', False), 
          ('ssr', '--singleStepSingleAction --dynamicReordering', 'Classic EnfPre, Reordering', 'color=red', False), 
          ('ssb', '--singleStepSingleAction', 'Classic EnfPre, Best Order', 'color=red,densely dotted', True), 
          ]

VARIANTS = [a for (a,b,c,d,e) in config]
STYLES = [d for (a,b,c,d,e) in config]
LABELS = [c for (a,b,c,d,e) in config]
MAX_TIME = 4200
TIME_STEPS_DRAW = [0.1,1,10,60,600,3600]
IMAGE_HEIGHT = 8.2
IMAGE_WIDTH = 13
NOF_XAXIS_TICKS = 12

def timeLogConverter(thetime):
    if (thetime)<0.05:
        return 0.0
    return IMAGE_HEIGHT/(math.log(MAX_TIME)-math.log(0.05))*(math.log(thetime)-math.log(0.05))  


benchmarkTimes = {}
nofBenchmarks = 0
for variant in VARIANTS:
    allBenchmarkFiles = glob.glob(".res/*.txt"+variant+".txt")
    allNumbers = []
    for filename in allBenchmarkFiles: 
        thisTime = None
        timeout = False
        with open(filename,"r") as dataFile:
            for line in dataFile:
                line = line.strip().split(" ")
                if line[0] == "FINISHED":
                    assert line[1]=="CPU"
                    thisTime = float(line[2])
                elif line[0]=="TIMEOUT":
                    timeout = True
        if not timeout:
            if thisTime is None:
                print("Warning: No time found in file "+filename,file=sys.stderr)
            else:
                allNumbers.append(thisTime)            

    allNumbers.sort()
    benchmarkTimes[variant] = allNumbers   
    nofBenchmarks = max(nofBenchmarks,len(allNumbers))
    
       
print("\\documentclass[halfparskip,DIV18]{scrartcl}")
print("\\usepackage{tikz}")
print("\\title{Benchmark results}")
print("\\begin{document}\n\\maketitle")
print("\\section{Cactus plot over "+str(nofBenchmarks)+" benchmarks}")
    
    
                
# Draw data
print("\\begin{tikzpicture}[xscale=1.0]")
print("\\draw[->] (0,0) -- (0,"+str(IMAGE_HEIGHT)+");")
print("\\draw[->] (0,0) -- ("+str(IMAGE_WIDTH+0.1)+",0);")
# print("\\node[draw,anchor=south west] at ("+str(IMAGE_WIDTH+1)+",0) { ")

# X Axis Description
stepping = IMAGE_WIDTH/float(nofBenchmarks)
benchStep = max(1,int((nofBenchmarks+1)/NOF_XAXIS_TICKS))

# Increase the number of benchmarks for drawing the figure until we have a multiple of the number of the BENCH_STEP
while (nofBenchmarks % benchStep)!=0:
    nofBenchmarks += 1
stepping = IMAGE_WIDTH/float(nofBenchmarks)

for i in range(0,nofBenchmarks+1,benchStep):
    data = str(i)
    data = "\\textbf{"+data+"}"
    print("\\draw ("+str(i*stepping)+",0.1) -- +(0,-0.2) node[below] {"+data+"};")
    if i>0:
        print("\\draw[color=black!50!white,dashed] ("+str(i*stepping)+",0) -- +(0,"+str(IMAGE_HEIGHT)+");")
    
# Y Axis Description
for i in TIME_STEPS_DRAW:
    y = timeLogConverter(i)
    data = str(i)
    print("\\draw (0.1,"+str(y)+") -- +(-0.2,0) node[left] {"+data+"};")  
    print("\\draw[color=black!50!white,dashed] (0.1,"+str(y)+") -- +("+str(IMAGE_WIDTH)+",0);")

# Draw data
for j,variant in enumerate(LABELS):
    points = benchmarkTimes[VARIANTS[j]]
    for i in range(0,len(points)):
        if i>0:
            print("-|")
        else:
            print ("\\draw["+STYLES[j]+"]")
        print("("+str(i*stepping)+","+str(timeLogConverter(points[i]))+")")
    print(";")
    
# Axis descriptions
print("\draw (0,"+str(IMAGE_HEIGHT)+") node[below right,yshift=-4pt,fill=white,inner sep=1pt] { Time (seconds)};")
print("\draw ("+str(IMAGE_WIDTH)+",0) node[above left,yshift=2pt,fill=white,inner sep=1pt] { Number of solved benchmarks };")

    
print("\\end{tikzpicture}\n\n")
    
# Paint legend
for i,a in enumerate(VARIANTS):
    print("{\\begin{tikzpicture}\\draw["+STYLES[i]+"] (0,0) -- (0.7,0);\\end{tikzpicture}$\\ $"+LABELS[i]+"} ")

print("\\end{document}")
