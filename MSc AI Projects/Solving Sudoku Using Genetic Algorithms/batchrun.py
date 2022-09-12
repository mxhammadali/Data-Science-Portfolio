import csv
import time
import os
import os.path as path
import subprocess

# usage: main.py [-h] [-s PARENT_SELECTION_STRATEGY] [-e EDGE_FITNESS] [-t TARGET_POPULATION_SIZE] [-m MIN_POPULATION_SIZE] [-g MAX_GENERATIONS]
#                [-c CHILD_MUTATION_RATE] [-w WORD] [-v VERBOSITY] [-n NUMBER_OF_GAMES] [-p NUMBER_OF_PARENTS] [-o OUTPUT_TYPE] [-i INITIAL_CONFIG]

# optional arguments:
#   -h, --help            show this help message and exit
#   -s PARENT_SELECTION_STRATEGY, --parent-selection-strategy PARENT_SELECTION_STRATEGY
#                         Parent selection strategy: TO for Tournament, RO for Roulette, RA for ranked
#   -e EDGE_FITNESS, --edge-fitness EDGE_FITNESS
#                         Require one or two edges to spell the symbols backwards or forwards
#   -t TARGET_POPULATION_SIZE, --target-population-size TARGET_POPULATION_SIZE
#                         Target number of chromosomes over all populations
#   -m MIN_POPULATION_SIZE, --min-population-size MIN_POPULATION_SIZE
#                         Minimum number of chromosomes in case of multiple populations (i.e. --edge-fitness)
#   -g MAX_GENERATIONS, --max-generations MAX_GENERATIONS
#                         Upper limit of generations to try finding perfectly fit chromosome
#   -c CHILD_MUTATION_RATE, --child-mutation-rate CHILD_MUTATION_RATE
#                         Probability of a child mutation after crossover
#   -w WORD, --word WORD  Word (sequence of symbols, aka alphabet) to be used, must strictly be 4 characters long
#   -v VERBOSITY, --verbosity VERBOSITY
#                         Verbosity: 0-Print out only statistics, 1-Print out progress. 2-Print full populations
#   -n NUMBER_OF_GAMES, --number-of-games NUMBER_OF_GAMES
#                         Play n games, whereas default is 1
#   -p NUMBER_OF_PARENTS, --number-of-parents NUMBER_OF_PARENTS
#                         Breed with that number of parents, whereas default is population size
#   -o OUTPUT_TYPE, --output-type OUTPUT_TYPE
#                         Output Type: N for No Formatting, C for CSV, M for Markdown
#   -i INITIAL_CONFIG, --initial-config INITIAL_CONFIG
#                         Number of default presets, allowed values are dict_keys([0, 4, 7])
cmd = 'python main.py'
arg_fmt = ' -p {p} -i {i} -c {c} -t {t} -s {s} -o {o} -v {v} -n {n}'
argsets = [
    [2,7,1.0,4,"TO","M",2,1000],
    [4,7,1.0,4,"TO","M",2,1000],
    [2,7,0.5,4,"TO","M",2,1000],
    [4,7,0.5,4,"TO","M",2,1000],
    [4,7,1.0,8,"TO","M",2,1000],
    [8,7,1.0,8,"TO","M",2,1000],
    [4,7,0.5,8,"TO","M",2,1000],
    [8,7,0.5,8,"TO","M",2,1000],
    [2,4,1.0,4,"TO","M",2,1000],
    [4,4,1.0,4,"TO","M",2,1000],
    [2,4,0.5,4,"TO","M",2,1000],
    [4,4,0.5,4,"TO","M",2,1000],
    [4,4,1.0,8,"TO","M",2,1000],
    [8,4,1.0,8,"TO","M",2,1000],
    [4,4,0.5,8,"TO","M",2,1000],
    [8,4,0.5,8,"TO","M",2,1000],
    [2,0,1.0,4,"TO","M",2,1000],
    [4,0,1.0,4,"TO","M",2,1000],
    [2,0,0.5,4,"TO","M",2,1000],
    [4,0,0.5,4,"TO","M",2,1000],
    [4,0,1.0,8,"TO","M",2,1000],
    [8,0,1.0,8,"TO","M",2,1000],
    [4,0,0.5,8,"TO","M",2,1000],
    [8,0,0.5,8,"TO","M",2,1000],
    [2,7,1.0,4,"RO","M",2,1000],
    [4,7,1.0,4,"RO","M",2,1000],
    [2,7,0.5,4,"RO","M",2,1000],
    [4,7,0.5,4,"RO","M",2,1000],
    [4,7,1.0,8,"RO","M",2,1000],
    [8,7,1.0,8,"RO","M",2,1000],
    [4,7,0.5,8,"RO","M",2,1000],
    [8,7,0.5,8,"RO","M",2,1000],
    [2,4,1.0,4,"RO","M",2,1000],
    [4,4,1.0,4,"RO","M",2,1000],
    [2,4,0.5,4,"RO","M",2,1000],
    [4,4,0.5,4,"RO","M",2,1000],
    [4,4,1.0,8,"RO","M",2,1000],
    [8,4,1.0,8,"RO","M",2,1000],
    [4,4,0.5,8,"RO","M",2,1000],
    [8,4,0.5,8,"RO","M",2,1000],
    [2,0,1.0,4,"RO","M",2,1000],
    [4,0,1.0,4,"RO","M",2,1000],
    [2,0,0.5,4,"RO","M",2,1000],
    [4,0,0.5,4,"RO","M",2,1000],
    [4,0,1.0,8,"RO","M",2,1000],
    [8,0,1.0,8,"RO","M",2,1000],
    [4,0,0.5,8,"RO","M",2,1000],
    [8,0,0.5,8,"RO","M",2,1000],
    [2,7,1.0,4,"RA","M",2,1000],
    [4,7,1.0,4,"RA","M",2,1000],
    [2,7,0.5,4,"RA","M",2,1000],
    [4,7,0.5,4,"RA","M",2,1000],
    [4,7,1.0,8,"RA","M",2,1000],
    [8,7,1.0,8,"RA","M",2,1000],
    [4,7,0.5,8,"RA","M",2,1000],
    [8,7,0.5,8,"RA","M",2,1000],
    [2,4,1.0,4,"RA","M",2,1000],
    [4,4,1.0,4,"RA","M",2,1000],
    [2,4,0.5,4,"RA","M",2,1000],
    [4,4,0.5,4,"RA","M",2,1000],
    [4,4,1.0,8,"RA","M",2,1000],
    [8,4,1.0,8,"RA","M",2,1000],
    [4,4,0.5,8,"RA","M",2,1000],
    [8,4,0.5,8,"RA","M",2,1000],
    [2,0,1.0,4,"RA","M",2,1000],
    [4,0,1.0,4,"RA","M",2,1000],
    [2,0,0.5,4,"RA","M",2,1000],
    [4,0,0.5,4,"RA","M",2,1000],
    [4,0,1.0,8,"RA","M",2,1000],
    [8,0,1.0,8,"RA","M",2,1000],
    [4,0,0.5,8,"RA","M",2,1000],
    [8,0,0.5,8,"RA","M",2,1000]
]

out_folder = "out"
try:
    os.makedirs(out_folder, exist_ok = True)
    print("Directory '%s' created successfully" % out_folder)
except OSError as error:
    print("Directory '%s' can not be created" % out_folder)
headers = ["Selection Strategy","Game Ordinal", "Population Size", "Preset fields", "Mutation Probability", "Parents", "Elites","Max Generations", "Average to Converge"]
filename_run_stats = "test-run-stats.csv"

filename_run_stats = path.join(out_folder,filename_run_stats)
with open(filename_run_stats,"w+") as gen_csv:
    gen_writer = csv.writer(gen_csv,delimiter=',')
    gen_writer.writerow(headers)

for argset in argsets:
    arg = arg_fmt.format(arg_fmt, p=argset[0], i = argset[1], c = argset[2], t = argset[3], s = argset[4], o = argset[5], v = argset[6], n=argset[7])
    p = subprocess.Popen(cmd + arg, stdout=subprocess.PIPE, shell=True)
    out, err = p.communicate()

    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = "{s}-{i}-{p}-{t}-{c}-{time}.md".format(p=argset[0], i = argset[1], c = argset[2], t = argset[3], s = argset[4], time=timestr)
    # folder_name = out_folder.format(timestr)
    filename = path.join(out_folder,filename)
    with open(filename, 'w') as f:
        f.write(out.decode("utf-8"))
   