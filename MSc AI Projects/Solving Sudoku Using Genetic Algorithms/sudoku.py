import numpy as np
class Sudoku:
    """
    This class provides basic Sudoku configuration handling
    """
    def __init__(self, word, initial_config, edge_fitness=False):
        self._word = word
        self._initial_config = initial_config
        self._edge_fitness = edge_fitness

    def initial_config(self):
        return self._initial_config

    def initial_configs(self):
        # Derive enriched initial configs with word along edges if configured to do so,
        # otherwise: just provide initial configuration given through constructor, but
        # provide empty array in case initial configuration is invalid
        if self._edge_fitness:
            return self._edge_configs()
        else:
            return np.array([self._initial_config if self._validate_config(self._initial_config) else []])

    def _edge_configs(self):
        # Method to create list of configurations that have given word along edges
        edge_configs = []

        # First and last row, word written backward and forward
        edge_configs.append(np.concatenate(([self._word], self._initial_config[1:4])))
        edge_configs.append(np.concatenate((self._initial_config[0:3], [self._word])))
        edge_configs.append(np.concatenate(([np.flip(self._word)], self._initial_config[1:4])))
        edge_configs.append(np.concatenate((self._initial_config[0:3], [np.flip(self._word)])))
        edge_configs.append(np.concatenate(([self._word], self._initial_config[1:3], [np.flip(self._word)])))
        edge_configs.append(np.concatenate(([np.flip(self._word)], self._initial_config[1:3], [self._word])))

        # First and last column, word written backward and forward
        swap_config = self._initial_config.swapaxes(0, 1)
        edge_configs.append(np.concatenate(([self._word], swap_config[1:4])).swapaxes(0, 1))
        edge_configs.append(np.concatenate((swap_config[0:3], [self._word])).swapaxes(0, 1))
        edge_configs.append(np.concatenate(([np.flip(self._word)], swap_config[1:4])).swapaxes(0, 1))
        edge_configs.append(np.concatenate((swap_config[0:3], [np.flip(self._word)])).swapaxes(0, 1))
        edge_configs.append(np.concatenate(([self._word], swap_config[1:3], [np.flip(self._word)])).swapaxes(0, 1))
        edge_configs.append(np.concatenate(([np.flip(self._word)], swap_config[1:3], [self._word])).swapaxes(0, 1))

        # Remove all configurations which are invalid or are not compliant with given initial configuration
        edge_configs = [cfg for cfg in edge_configs if self._validate_config(cfg) and self._compliant_config(cfg)]
        print([e for e in edge_configs])
        return np.array(edge_configs)

    def _validate_config(self, sudoku_config):
        # Support method to validate if an initial Sudoku configuration is valid
        # First check if only valid elements are included
        if np.setdiff1d(sudoku_config.flatten(), np.append(self._word, '-')).size != 0:
            return False

        # Then check if any row, column or sub-block has duplicate symbols
        slices = sudoku_config
        slices = np.append(slices, np.swapaxes(slices, 0, 1), 0)
        slices = np.append(slices, np.reshape(np.reshape(sudoku_config, (4, 2, 2)).swapaxes(0, 1), (4, 4)), 0)
        for slice in slices:
            test_array = np.delete(slice, np.where(slice == '-'))
            if not np.array_equal(np.unique(test_array), np.sort(test_array)):
                return False

        return True

    def _compliant_config(self, sudoku_config):
        # Support method to validate if an enriched Sudoku config is compliant with the initial configuration
        initial_config_mask = [True if pos in self._word else False for pos in self._initial_config.flatten()]
        return np.array_equal(sudoku_config.flatten()[initial_config_mask], self._initial_config.flatten()[initial_config_mask])
