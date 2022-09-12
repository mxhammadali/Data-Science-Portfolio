import numpy as np
import config
import random
import time
import csv
import os
import os.path as path
import markdown_strings as md
from itertools import zip_longest
from sudoku import Sudoku
class Chromosome:
    def __init__(self, initial_config, chromosome=None):
        ### Add generation+nbr to identify surviving parents etc...
        # Reformat new chromosome or create one from initial configuration, with block-preserving format
        initial_config = np.reshape(initial_config.flatten(), (4, 2, 2)).swapaxes(0, 1).reshape((4, 4))
        if chromosome is None:
            self._chromosome = np.copy(initial_config)
        else:
            self._chromosome = np.reshape(chromosome.flatten(), (4, 2, 2)).swapaxes(0, 1).reshape((4, 4))
        for block in self._chromosome:
            missing_symbols = np.setdiff1d(config.symbols(), block)
            np.random.shuffle(missing_symbols)
            np.put(block, np.where(block == '-')[0], missing_symbols)
        self._update_fitness()

        # Pre-calculate free positions for later mutation; for more efficiency: Put this into sudoku class (future)
        self._position_choices = []
        for block in range(4):
            if len(set(initial_config[block])) < 4:
                self._position_choices.append((block, np.where(initial_config[block] == '-')[0]))

    def initial_config(self):
        # Get initial config of chromosome
        return self.initial_config

    def chromosome(self):
        # Get chromosome view
        return self._chromosome

    def row_view(self):
        # Standard Sudoku row view
        return np.reshape(self._chromosome, (2, 4, 2)).swapaxes(1, 0).reshape((4, 4))

    def column_view(self):
        # Sudoku column view
        return self.row_view().swapaxes(0, 1)

    @staticmethod
    def _row_fitness(rows):
        # Calculate total row fitness
        unique_values = 0
        for row in rows:
            unique_values += len(set(row))
        return unique_values

    @staticmethod
    def get_fitnesses(rows):
        fitnesses = []
        for row in rows:
            fitnesses.append(len(set(row)))
        return fitnesses

    def _total_row_fitness(self):
        # Calculate total row fitness
        return self._row_fitness(self.row_view())

    def _total_column_fitness(self):
        # Calculate total column fitness
        return self._row_fitness(self.column_view())

    def _update_fitness(self):
        # Calculate total fitness
        self._top_fitness = self._row_fitness(self.row_view()[:2]) / 8
        self._bottom_fitness = self._row_fitness(self.row_view()[2:]) / 8
        self._left_fitness = self._row_fitness(self.column_view()[:2]) / 8
        self._right_fitness = self._row_fitness(self.column_view()[2:]) / 8
        self._fitness = (self._top_fitness + self._bottom_fitness + self._left_fitness + self._right_fitness) / 4

    def partial_fitness(self):
        # Return partial fitness tuple
        return self._top_fitness, self._bottom_fitness, self._left_fitness, self._right_fitness

    def total_fitness(self):
        # Return total fitness
        return self._fitness

    def mutate(self):
        # Mutate by swapping two symbols in a block which has at least two positions undefined in the initial config
        index = random.randint(0, len(self._position_choices) - 1)
        block = self._position_choices[index][0]
        swap = np.random.choice(self._position_choices[index][1], 2, replace=False)
        self._chromosome[block][swap[0]], self._chromosome[block][swap[1]] = \
            self._chromosome[block][swap[1]], self._chromosome[block][swap[0]]
        self._update_fitness()

    def to_markdown(self):
        print(md.table_row([' ','','','']))
        print(md.table_delimiter_row(4))
        for row in self.row_view():
            print(md.table_row(row))
        print(md.table_row(['Fitness:', '{: >7.2%} '.format(self.total_fitness())]))
        print()

    def to_string(self):
        for row in range(4):
            for symbol in self.row_view()[row]:
                print(symbol, end=' ')
            print(' | ', end='')
        print('{: >7.2%}  | '.format(self.total_fitness()), end='')
        print()

    def __str__(self):
        return str(self.row_view())
class Population:
    def __init__(self, population_size, initial_config, population_number, game_number):
        self._initial_config = initial_config
        self._population_size = population_size
        self._population_number = population_number
        self._game_number = game_number
        self._generation_number = 0
        self._chromosomes = [Chromosome(initial_config) for x in range(population_size)]

    def _parent_selection(self, n):
        strategy = config.parent_selection_strategy()
        if strategy == config.PARENT_SELECTION_TOURNAMENT:
            parents = self._tournament_selection(n)
        elif strategy == config.PARENT_SELECTION_ROULETTE:
            parents = self._roulette_wheel_selection(n)
            parents += self._roulette_wheel_selection(n)
        elif strategy == config.PARENT_SELECTION_RANKED:
            parents = self._ranked_selection(n)
        return zip(*[iter(parents)] * 2)

    def _tournament_selection(self, n):
        parents = []
        for _ in range(n):
            sample = random.sample(self._chromosomes, min(self._population_size // 2, 50))
            parents.append(max(sample, key=lambda x: x.total_fitness()))
        return parents

    def _roulette_wheel_selection(self, n):
        # Bear with us: Python much more compact than pseudo code representation of algorithm
        return random.choices(self._chromosomes, weights=[c.total_fitness() for c in self._chromosomes], k=n)

    def _ranked_selection(self, n):
        self._chromosomes.sort(key=lambda c: c.total_fitness())
        return random.choices(self._chromosomes, weights=[i for i in range(len(self._chromosomes))], k=n)

    def _crossover(self, parent1, parent2):
        parent1_fitness = parent1.partial_fitness()
        parent2_fitness = parent2.partial_fitness()

        child1_chromosome = np.concatenate((
            parent1.row_view()[:2] if parent1_fitness[0] >= parent2_fitness[0] else parent2.row_view()[:2],
            parent1.row_view()[2:] if parent1_fitness[1] >= parent2_fitness[1] else parent2.row_view()[2:]
        ))

        child2_chromosome = np.concatenate((
            parent1.column_view()[:2] if parent1_fitness[2] >= parent2_fitness[2] else parent2.column_view()[:2],
            parent1.column_view()[2:] if parent1_fitness[3] >= parent2_fitness[3] else parent2.column_view()[2:])
        ).swapaxes(1, 0)

        return Chromosome(self._initial_config, child1_chromosome), Chromosome(self._initial_config, child2_chromosome)

    def _survivor_selection(self):
        self._chromosomes.sort(key=lambda c: c.total_fitness(), reverse=True)
        self._chromosomes = self._chromosomes[0:self._population_size]

    def fittest_chromosome(self):
        return max(self._chromosomes, key=lambda c: c.total_fitness())

    def next_generation(self):
        self._generation_number += 1
        parents = self._parent_selection(min(config.number_of_parents(), len(self._chromosomes)))
        for parent1, parent2 in parents:
            child1, child2 = self._crossover(parent1, parent2)
            if random.randint(1, 100) <= config.child_mutation_probability() * 100:
                child1.mutate()
                child2.mutate()
            self._chromosomes.append(child1)
            self._chromosomes.append(child2)
        self._survivor_selection()

    def print(self):
        if config.output_type() == config.MARKDOWN_OUTPUT:
            self.to_markdown()
        else:
            self.to_string()
    
    def get_stats(self):
        i = 0
        stats = []
        for chromosome in self._chromosomes:
            pop_no = self._population_number
            chr_no = i
            total_fit = chromosome.total_fitness()
            pop_size = self._population_size
            game = self._game_number
            gen = self._generation_number
            stats.append([game,gen,pop_no,chr_no,pop_size,total_fit])
            i+= 1
        return stats

    def to_markdown(self):
        rows = np.array(self._chromosomes).reshape(-1,2);
        print(md.table_row(['Population', '{num}'.format(num=self._population_number),'','','','','','','','','']))
        print(md.table_row([":---:"] * 11))
        i = 0
        for row in rows:
            lrf = Chromosome.get_fitnesses(row[0].row_view())
            rrf = Chromosome.get_fitnesses(row[0].row_view())
            lcf = Chromosome.get_fitnesses(row[0].column_view())
            rcf = Chromosome.get_fitnesses(row[0].column_view())
            ltf = row[0].total_fitness()
            rtf = row[1].total_fitness()
            print(md.table_row(['Chromosome {num}'.format(num=i +1 ),'','','','','','Chromosome {num}'.format(num=i +2 ),'','','','']))
            for line in range(4):
                l = row[0].row_view()[line]
                r = row[1].row_view()[line]
                lf = lrf[line]
                l = np.append(l,[lf])
                rf = rrf[line]
                r = np.append(r,[rf])
                n = np.append(l,np.append([""],r))
                print(md.table_row(n))
            col_fitness = lcf + ["{:.0%}".format(ltf),""] + rcf + ["{:.0%}".format(rtf)]
            
            print(md.table_row(col_fitness))
            print(md.table_row([''] * 11))
            i += 2
        print()

    def to_string(self):
        pop = 'Population {num}'.format(num=self._population_number)
        print(pop)
        for chromosome in self._chromosomes:
            chromosome.to_string()
        print()
class Populations:
    def __init__(self, n, initial_configs, game):
        # Create a population for all initial Sudoku configurations
        self._game = game
        self._populations = []
        _population_counter = 0
        for initial_config in initial_configs:
            _population_counter += 1
            self._populations.append(Population(n, initial_config, _population_counter, game))

    def next_generation(self):
        # Create a next generation for all populations
        for population in self._populations:
            population.next_generation()

    def print(self):
        # Print all chromosomes of all populations
        if config.verbosity() == config.MAX_VERBOSITY:
            if self._game == 0 :
                for population in self._populations:
                    population.print()

    def get_stats(self):
        stats = []
        for population in self._populations:
            for st in population.get_stats():
                stats.append(st)
        return stats

            
    def fittest_chromosome(self):
        # Return fittest chromosome from the total set of chromosomes of all populations
        fittest_chromosomes = [p.fittest_chromosome() for p in self._populations]
        return max(fittest_chromosomes, key=lambda c: c.total_fitness())

def genesis(game_number):
    # Set-up initial Sudoku configuration and (potentially) enriched edge configurations
    sudoku = Sudoku(config.symbols(), config.initial_config(), config.edge_fitness())
    initial_configs = sudoku.initial_configs()
    nbrs_of_pupulations = len(initial_configs)
    stats = []
    if nbrs_of_pupulations == 0:
        print('Impossible starting configuration or requirements')
        exit()

    # Create initial populations
    population_size = max(config.min_population_size(), config.target_population_size() // nbrs_of_pupulations)
    if game_number == 0:
        print_populationinfo(nbrs_of_pupulations, population_size)
    populations = Populations(population_size, initial_configs,game_number)
    if game_number == 0:
        print_header("Generation 0", 2)
    populations.print()
    for st in populations.get_stats():
        stats.append(st)
    generation = 0

    # Grow next generations until we have a perfectly fitting chromosome OR we exceed configured max generations
    while populations.fittest_chromosome().total_fitness() < 1 and generation < config.max_generations():
        generation += 1
        if game_number == 0:
            print_header("Generating generation {:d}".format(generation),2)
        populations.next_generation()
        populations.print()
        for st in populations.get_stats():
            stats.append(st)

        if game_number == 0:
            print_fittest(populations)

    if game_number == 0:
        print_fin(generation, populations)

    return generation, stats

def print_fin(generation, populations):
    print('Finished in generation {:d}, best solution fitness = {:.2%}\n{}'.format(
          generation, populations.fittest_chromosome().total_fitness(), populations.fittest_chromosome().to_markdown()))

def print_fittest(populations):
    print("Fittest chromosome", populations.fittest_chromosome().total_fitness())
    print()

# txt = "Generating generation {:d}".format(generation)
def print_header(txt, level):   
    if config.output_type() == config.MARKDOWN_OUTPUT:
        print(md.header(txt,level))
        print()
    else:
        print(txt)
        print()

def print_md_hr():
    print(md.horizontal_rule())

def grouper(iterable, n, fillvalue=None):
    "Collect data into fixed-length chunks or blocks"
    args = [iter(iterable)] * n
    return zip_longest(*args, fillvalue=fillvalue)

def print_populationinfo(nbrs_of_pupulations, population_size):
    info = 'Creating {:d} population(s) with {:d} chromosomes each.'.format(nbrs_of_pupulations, population_size)
    print_header(info,2)

if __name__ == '__main__':
    # print('Configuration:', vars(config.args))
    max_generation = 0 # config.max_generations()
    total_generations = 0
    highest_generations = 0
    stats = []
    run_stats = []
    timestr = time.strftime("%Y%m%d-%H%M%S")
    t = "{}".format(config.target_population_size())
    ic = "{}".format(config.initial_config_option())
    c = "{}".format(config.child_mutation_probability())
    p = "{}".format(config.number_of_parents())
    ne = "{}".format(config.target_population_size() - config.number_of_parents())
    ss = config.parent_selection_strategy()
    stats_file_header = ["Selection Strategy","Game Ordinal","Parents" , "Preset fields", "Mutation Probability", "Population Size", "Elites","Game Generations"]
    filename_generations = "{s}-{i}-{p}-{t}-{c}-gen-{time}.csv".format(p=p, i = ic, c = c, t = t, s = ss , time=timestr)
    filename_stats = "{s}-{i}-{p}-{t}-{c}-stats-{time}.csv".format(p=p, i = ic, c = c, t = t, s = ss, time=timestr)
    filename_run_stats = "test-run-stats.csv"

    out_folder = "out"
    try:
        os.makedirs(out_folder, exist_ok = True)
        print("Directory '%s' created successfully" % out_folder)
    except OSError as error:
        print("Directory '%s' can not be created" % out_folder)
    filename_stats = path.join(out_folder,filename_stats)
    filename_generations = path.join(out_folder,filename_generations)
    filename_run_stats = path.join(out_folder,filename_run_stats)

    with open(filename_stats,"w+") as stats_csv:
        stats_writer = csv.writer(stats_csv,delimiter=',')
        stats_writer.writerow(stats_file_header)
    for i in range(config.number_of_games()):
        game_ordinal = i + 1
        if i == 0:
            print_header('Game #{}'.format(i),1)

        generations, game_stats = genesis(i)
        for st in game_stats:
            stats.append(st)
        if generations > max_generation:
            max_generation = generations
        total_generations += generations
        if config.number_of_games() > 1:
            gg = generations
            mg = "{}".format(max_generation)
            atc = "{}".format(total_generations / config.number_of_games())
            game_results = [ss, game_ordinal, p, ic, c , t, ne, gg]
            run_stats = [ss, game_ordinal, p, ic, c , t, ne ,mg , atc]
            if i == 0:
                print()
                print()
                print(md.table_row(stats_file_header))
                print(md.table_row([":---"] * len(stats_file_header)))
            print(md.table_row(game_results))
            with open(filename_stats,"a") as stats_csv:
                stats_writer = csv.writer(stats_csv,delimiter=',')
                stats_writer.writerow(game_results)
            
    
    with open(filename_generations,"w+") as gen_csv:
        gen_writer = csv.writer(gen_csv,delimiter=',')
        gen_writer.writerow(["Game Number","Generation","Population","Chromosome","Populdation Size","Fitness"])
        gen_writer.writerows(stats)

    with open(filename_run_stats,"a") as run_csv:
        run_writer = csv.writer(run_csv,delimiter=',')
        run_writer.writerow(run_stats)