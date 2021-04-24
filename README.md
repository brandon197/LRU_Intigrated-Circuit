# LRU_Intigrated-Circuit
A VLSI design of the Least Recently Used algorithm




In modern computing, the combination of many page replacement algorithms has
contributed to a significant increase in cache performance. The Least Recently Used (LRU)
algorithm works by keeping track of how often a cache way is accessed and uses the least
accessed way in a set for page replacement. Doing so allows heavily used pages that were
used in the past few instructions to be available in the next few instructions as it is more
likely that they will be needed.

The goal of our project is to create an integrated circuit (IC) that replicates the nature of the algorithm for a
4-way set-associative cache that has a total of 8 sets within a 600nM process. This incluedes creating verilog simulations, schematics and layouts In doing so
we will keep track of how often each way in each set is accessed by using a 2-bit counter
where the value 11 represents the most recently accessed and 00 represents the least
recently accessed.

Tools used are Cadence Virtuoso, Innovus and Design Compiler

![image](https://user-images.githubusercontent.com/46882012/115946521-bb9f7400-a48f-11eb-9e52-1ce593e0f26f.png)

The project library and all of its view cells made in Virtuoso are in a sub-folder named project-library in this repo.
The initial and final lru_chip architecture diagrams mentioned in section 4 are in two jpeg
files in the submitted package named:
● Initial_lru_chip_architecture.jpeg
● Final_lru_chip_architecture.jpeg.

verilog_test is a sub-folder in the submitted package that contains the three files that are
required to test the verilog code. Simply open a terminal on the verilog_test directory and run
sim-nc lru_unit.sv.

The chip plot can also be found in the root folder of the submitted package named Chip_plot.pdf

Final report is in the pdf.
