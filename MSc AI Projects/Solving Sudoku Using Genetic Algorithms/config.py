import numpy as np
import argparse

# Default arguments used in case nothing given via commandline
__NUMBER_OF_PARENTS = 8            # Choose that many parents, but no more than total population size
__TARGET_POPULATION_SIZE = 16      # Target population size over all populations within a generation
__MIN_POPULATION_SIZE = 8          # Used if different starting configurations can be derived in case __EDGE_FITNESS
__MAX_GENERATIONS = 10000           # Weird high value, just to stop from endless loop during development/tests
__CHILD_MUTATION_RATE = 1.0        # Chromosome and algorithm design favors a high child mutation rate
__PARENT_SELECTION_STRATEGY = "RA" # "TO" (Tournament), "RO" (Roulette), "RA" (Ranked)
__WORD = 'WORD'                    # Sequence of symbols
__EDGE_FITNESS = False             # Require one or two edges to spell the symbols backwards or forwards
__VERBOSITY = 1                    # 0: Print out only statistics. 1: Print out progress. 2: Print full populations
__OUTPUT_TYPE = "N"                # N: Default output, C: CSV, M: Markdown
__NUMBER_OF_GAMES = 1              # Default is to play 1 game

__INITIAL_CONFIGURATION_ZERO = [['-', '-', '-', '-'],
                                ['-', '-', '-', '-'],
                                ['-', '-', '-', '-'],
                                ['-', '-', '-', '-']]
__INITIAL_CONFIGURATION_FOUR = [['-', 'R', '-', '-'],
                                ['-', '-', '-', '-'],
                                ['-', '-', '-', 'O'],
                                ['D', '-', '-', 'W']]
__INITIAL_CONFIGURATION_SEVEN =[['-', 'O', '-', '-'],
                                ['-', '-', 'W', 'O'],
                                ['O', 'W', '-', '-'],
                                ['-', '-', 'O', 'W']]

# Do not mess with this: Only supported parent selection strategies
PARENT_SELECTION_TOURNAMENT = "TO"
PARENT_SELECTION_RANKED = "RA"
PARENT_SELECTION_ROULETTE = "RO"
__INITIAL_CONFIGURATIONS = {
    0: __INITIAL_CONFIGURATION_ZERO,
    4: __INITIAL_CONFIGURATION_FOUR,
    7: __INITIAL_CONFIGURATION_SEVEN
}
__PARENT_SELECTION_STRATEGY_OPTIONS = [
    PARENT_SELECTION_TOURNAMENT,
    PARENT_SELECTION_RANKED,
    PARENT_SELECTION_ROULETTE,
]
MAX_VERBOSITY = 2
MEDIUM_VERBOSITY = 1
MIN_VERBOSITY = 0
DEFAULT_OUTPUT = "N"
CSV_OUTPUT = "C"
MARKDOWN_OUTPUT = "M"
__OUTPUT_TYPE_OPTIONS = [
    DEFAULT_OUTPUT,
    CSV_OUTPUT,
    MARKDOWN_OUTPUT,
]
# This module behaves like a singleton, as there should not be multiple configuration class instances

def target_population_size():
    return args.target_population_size

def min_population_size():
    return args.min_population_size

def parent_selection_strategy():
    return args.parent_selection_strategy.upper()

def edge_fitness():
    return args.edge_fitness

def max_generations():
    return args.max_generations

def child_mutation_probability():
    return args.child_mutation_rate

def initial_config_option():
    return args.initial_config

def initial_config():
    return np.array(__INITIAL_CONFIGURATIONS[args.initial_config])

def symbols():
    return np.array(list(args.word))

def verbosity():
    return args.verbosity

def output_type():
    return args.output_type.upper()

def number_of_parents():
    return args.number_of_parents

def number_of_games():
    return args.number_of_games

def args():
    return args

# Parse commandline arguments upon initalisation
parser = argparse.ArgumentParser()
parser.add_argument("-s", "--parent-selection-strategy",
                    type=lambda s: s if s.upper() in __PARENT_SELECTION_STRATEGY_OPTIONS
                    else False, default=__PARENT_SELECTION_STRATEGY,
                    help="Parent selection strategy: TO for Tournament, RO for Roulette, RA for ranked")
parser.add_argument("-e", "--edge-fitness", type=bool, default=__EDGE_FITNESS,
                    help="Require one or two edges to spell the symbols backwards or forwards")
parser.add_argument("-t", "--target-population-size", type=int, default=__TARGET_POPULATION_SIZE,
                    help="Target number of chromosomes over all populations")
parser.add_argument("-m", "--min-population-size", type=int, default=__MIN_POPULATION_SIZE,
                    help="Minimum number of chromosomes in case of multiple populations (i.e. --edge-fitness)")
parser.add_argument("-g", "--max-generations", type=int, default=__MAX_GENERATIONS,
                    help="Upper limit of generations to try finding perfectly fit chromosome")
parser.add_argument("-c", "--child-mutation-rate", type=float, default=__CHILD_MUTATION_RATE,
                    help="Probability of a child mutation after crossover")
parser.add_argument("-w", "--word", type=lambda s: s if s.isalpha() and len(set(list(s))) == 4 else False, default=__WORD,
                    help="Word (sequence of symbols, aka alphabet) to be used, must strictly be 4 characters long")
parser.add_argument("-v", "--verbosity", type=int, default=__VERBOSITY,
                    help="Verbosity: 0-Print out only statistics, 1-Print out progress.  2-Print full populations")
parser.add_argument("-n", "--number-of-games", type=int, default=__NUMBER_OF_GAMES,
                    help="Play n games, whereas default is 1")
parser.add_argument("-p", "--number-of-parents", type=int, default=__NUMBER_OF_PARENTS,
                    help="Breed with that number of parents, whereas default is population size")
parser.add_argument("-o", "--output-type",
                    type=lambda s: s if s.upper() in __OUTPUT_TYPE_OPTIONS
                    else False, default=__OUTPUT_TYPE,
                    help="Output Type: N for No Formatting, C for CSV, M for Markdown")
parser.add_argument("-i", "--initial-config", type=int, default=0,
                    help="Number of default presets, allowed values are {}".format(__INITIAL_CONFIGURATIONS.keys()))

                    
args = parser.parse_args()

if not args.word:
    print('Invalid -w/-word option, must be string of four different characters.')
    exit(-1)

if not args.parent_selection_strategy:
    print('Invalid -p/--parent-selection-strategy, must be one of {}'.format(__PARENT_SELECTION_STRATEGY_OPTIONS))
    exit(-1)

if not args.output_type:
    print('Invalid -o/--output-type, must be one of {}'.format(__OUTPUT_TYPE_OPTIONS))
    exit(-1)

if not args.initial_config in __INITIAL_CONFIGURATIONS:
    print('Invalid -i/--initial-config, must be one of {}'.format(__INITIAL_CONFIGURATIONS.keys()))
    exit(-1)
