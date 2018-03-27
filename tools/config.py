#
#
#
#
# Config submodule
CONFIG = []

for (ext,param,tablecolumn,lineStyle) in [("mm","--multiStepMultiAction","M/M","color=yellow!70!black"), ("sm","--singleStepMultiAction","S/M","color=green!90!black"), ("ms","--multiStepSingleAction","M/S","color=blue"),
("ss","--singleStepSingleAction","S/S","color=red"), ("msc","--multiStepSingleActionMultiCompose","M/S+C","color=black")]:
    for reodering in [False,True,"best"]:
        if reodering=="best":
            extb = ext+"b"
            paramb = param
            tablecolumnb = tablecolumn + "/B"
            lineStyleB = lineStyle+",very thin"
            chosenOrder = True        
        elif reodering:
            extb = ext+"r"
            paramb = param + " --dynamicReordering"
            tablecolumnb = tablecolumn + "/R"
            lineStyleB = lineStyle
            chosenOrder = False
        else:
            extb = ext
            paramb = param
            tablecolumnb = tablecolumn
            lineStyleB = lineStyle + ",thick"
            chosenOrder = False
        for multiAnd in [False,True]:
            if not multiAnd or ext!="ss":
                if multiAnd:
                    extc = extb + "a"
                    paramc = paramb + " --andAbstractGrouping"
                    tablecolumnc = tablecolumnb + "/A"
                    lineStyleC = lineStyleB + ",dashed"
                else:
                    extc = extb
                    paramc = paramb
                    tablecolumnc = tablecolumnb
                    lineStyleC = lineStyleB
                CONFIG.append((extc,paramc,tablecolumnc,lineStyleC,chosenOrder))


