Bestirs -- Bdd-basEd game Solving for Translation InvaRiant System dynamics
---------------------------------------------------------------------------
Bestirs is a tool for generalized Buchi game solving for games built from abstractions.

License
-------

Bestirs is available under GPLv3 license.

Preparation:
------------

The bestirs tool is written in C++. Before the tool can be used, some tools/libraries need to be downloaded:

- The CUDD decision diagram library by [Fabio Somenzi](http://vlsi.colorado.edu/~fabio/), version 3.0
- The SCOTS library by [Matthias Rungger](...)

All of them can be obtained using the following command sequence (tested on Ubuntu Linux v.17.10)

    cd lib; wget ftp://vlsi.colorado.edu/pub/cudd-3.0.0.tar.gz; tar -xvzf cudd-3.0.0.tar.gz; cd ..
    cd lib; git clone https://gitlab.lrz.de/matthias/SCOTSv0.2.git; cd SCOTSv0.2; git checkout ff25c857d0d7e43cfc4296b7be2600f0d8b04626; cd ../..
    cd tools; wget https://raw.githubusercontent.com/pshved/timeout/edb59c93c167c15ede5ccc2795e1abee25ebf9b4/timeout; chmod +x timeout; cd ..

Please also visit the home pages of these tools.

The following commands build the main tool and some auxiliary tools:
    
    cd src/Solver; make -f DirectMakefile; cd ../..
    cd src/ExampleAbstractionMoonLander; make -f DirectMakefile; cd ../..
    cd src/ExampleAbstractionSingleSpeedVehicle; make -f DirectMakefile; cd ../..
    cd src/ExampleAbstractionVehicleSCOTS; make -f DirectMakefile; cd ../..
    

Visualizing the Example System Dynamics
---------------------------------------
If a Python interpreter is available and the pygame package installed, the three example dynamics can be simulated using the visualizer scripts provided with the dynamics. To run them, switch to any of the +ExampleAbstraction+ folders in the +src+ directory and run +./visualizer.py+. Note that the computation may take a while. The simulation is multi-threaded: one thread paints the visualization, while the other thread reads the next next actions from the game solver. As the latter step involves operations over binary decision diagrams (BDDs), this can take a while, which can slow down the simulation. The simulation can be paused and unpaused using the SPACE key. Pausing the simulation allows the buffer for the next steps of the execution to fill, which allows a smoother simulation after unpausing. Please terminate the visualizer by closing the window. Any other form of termination (also in case of error -- e.g., when the abstraction generator or the game solver were not compiled before running the simulation) may leave the visualizer in a state from which it needs to be killed with the SIGKILL signal. Since the visualizer does not write to any files, this is safe to do.


Running be benchmarks
---------------------
A couple of benchmarks for the game solver can be found in the +benchmarks+ folder. To re-compute the results, you can run the following commands:

    cd benchmarks
    ../tools/make_benchmark_makefile.py
    make -j<number of processors that need to have 3 GB available each>
    ../tools/make_benchmark_makefile.py
    make -j<number of processors that need have 3 GB available each>

The file "cactus.pdf" in the benchmark folder will then contain the final benchmarking results.
    
