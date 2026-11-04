# Script to combien datasets in the yolov11 format

# Assumes a directory structure of:
#   Dataset  -> data.yaml
#            \-> Train
#                      \-> Images
#                      \-> Labels
#            \-> Valid
#                      \-> Images
#                      \-> Labels
#
# This directory structure must be followed for optimal results.
#
# Usage:
#   python3 combine_datasets.py <name> <dataset1> <dataset2> ... <datasetN>
#
# note: this should be run from within the datasets directory. ensure that class names are unique.
#
from pathlib import Path


import os
import sys
import yaml
import subprocess


def assertDirectoryPathValid(path: Path, exist_prop: bool) -> None:
    """
        Raises FileNotFoundError the directory path given
        does not point to a directory.

        Arguments:
            exist_prop: bool (True if the directory should exist, False otherwise)
    """
    
    if os.path.isdir(path) == exist_prop:
        raise FileNotFoundError(f"{path} is not a directory.")


def assertDatasetValid(path: Path) -> None:
    """
        Checks conditions for dataset to be valid.
        Exits the program with an error for an invalid dataset.
    """

    dataset_path = Path('datasets') / path
    try:
        assertDirectoryPathValid(dataset_path, True)
        with open(dataset_path / "data.yaml", 'r') as file:
            data = yaml.safe_load(file)
            assertDirectoryPathValid(dataset_path / str(data['path']), True)
            assertDirectoryPathValid(dataset_path / str(data['train']), True)
            assertDirectoryPathValid(dataset_path / str(data['valid']), True)
    except FileNotFoundError as e:
        print(e.strerror)
        sys.exit(e.errno)
        

def assertArgsValid() -> tuple[Path, list[Path]]:
    """
        Checks if the arguments given to the script are valid.
        If they are invalid, raises the appropriate error and
        exits the program with an error code.
    """
    if len(sys.argv) < 4:
        raise ValueError("Usage: python3 combine_datasets.py <name> <dataset1> <dataset2> ... <datasetN>")
    name = Path(sys.argv[1])
    assertDirectoryPathValid(name, False)
    datasets = [Path(x) for x in sys.argv[2:]]
    for dataset in datasets:
        assertDatasetValid(dataset)
    return (name, datasets)
    

def combineDatasets(name: Path, datasets: list[Path]) -> None:
    """
        Combines all datasets in datasets list into new usable dataset
        stored on path name.
    """
    all_classes = {}
    class_count = 0
    # make new directory
    new_dataset_path = Path('datasets') / name
    new_dataset_train = new_dataset_path / 'train'
    new_dataset_valid = new_dataset_path / 'valid'
    os.mkdir(new_dataset_path)
    os.mkdir(new_dataset_train)
    os.mkdir(new_dataset_valid)
    for dataset in datasets:
        with open(dataset / 'data.yaml', 'r') as f:
            data = yaml.safe_load(f)
        dataset_classes = data['name']
        for c in dataset_classes:
            if not all_classes.get(c):
                all_classes[c] = class_count
        # copy the files to the new path
        dataset_path = dataset / str(data['path'])
        train_path = dataset_path  / str(data['train'])
        valid_path = dataset_path / str(data['valid'])
        subprocess.run(['cp', str(train_path) + '/*', '-t', str(new_dataset_train)])
        subprocess.run(['cp', str(valid_path) + '/*', '-t', str(new_dataset_valid)])
        # Use sed to fix class numbers in folder
        sed_command = ''
        for i in range(len(dataset_classes)):
            sed_command += f"s/^{i}/{all_classes[dataset_classes[i]]}/;" + sed_command
        for filename in os.listdir(train_path):
            subprocess.run(['sed', '-E', sed_command, str(new_dataset_train / filename), '-i'])
        for filename in os.listdir(valid_path):
            subprocess.run(['sed', '-E', sed_command, str(new_dataset_valid / filename), '-i'])
                

def main():
    name, datasets = assertArgsValid()
    combineDatasets(name, datasets)


if __name__ == "__main__":
    main()
            