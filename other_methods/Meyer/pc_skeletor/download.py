#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright (c) 2022 Lukas Meyer
Licensed under the MIT License.
See LICENSE file for more information.
"""

# Built-in/Generic Imports
# ...

# Libs
import os
from zipfile import ZipFile
import urllib.request
import glob

from tqdm import tqdm

# Own modules
# ...

EXISTS = True
NON_EXIST = False


class DownloadProgressBar(tqdm):
    def update_to(self, b=1, bsize=1, tsize=None):
        if tsize is not None:
            self.total = tsize
        self.update(b * bsize - self.n)


def download(url: str, output_dir: str, overwrite: bool = False):
    filename = os.path.join(output_dir, url.split('/')[-1])

    if os.path.exists(filename) and not overwrite:
        print('{} already exists in {}'.format(url.split('/')[-1], output_dir))
    else:
        with DownloadProgressBar(unit='B',
                                 unit_scale=True,
                                 miniters=1,
                                 desc=url.split('/')[-1]) as t:
            urllib.request.urlretrieve(url, filename=filename, reporthook=t.update_to)

    return filename


def extract(filename: str, output_dir: str):
    # opening the zip_file file in READ mode
    with ZipFile(filename, 'r') as zip_file:
        # printing all the contents of the zip_file file
        # zip_file.printdir()

        # extracting all the files
        print('Extracting all the files now...')
        zip_file.extractall(path=output_dir)
        print('Done!')


class Dataset:
    def __init__(self):
        self.dataset_name = None
        self.dataset_path = None
        self.filename = None
        self.url = None
        self.data_path = None
        self.scale = None  # in cm

    def __check_existence(self, output_directory, dataset_name):
        if output_directory == os.path.abspath(__file__):
            self.data_path = os.path.abspath(os.path.join(output_directory, '..', '..', 'data'))
        else:
            self.data_path = os.path.join(output_directory, 'data')

        os.makedirs(self.data_path, exist_ok=True)

        if os.path.exists(os.path.join(self.data_path, dataset_name)):
            return EXISTS
        else:
            return NON_EXIST

    def download_tree_dataset(self, output_path: str = os.path.abspath(__file__), overwrite: bool = False):

        self.url = 'https://faubox.rrze.uni-erlangen.de/dl/fiY7DMQ5TgQwoA1LvedgRu/tree.zip'

        self.dataset_name = 'tree'

        existence = self.__check_existence(output_directory=output_path, dataset_name=self.dataset_name)

        if existence == NON_EXIST:
            self.filename = download(url=self.url, output_dir=self.data_path, overwrite=overwrite)
            extract(filename=self.filename, output_dir=self.data_path)
        else:
            print('Dataset {} already exists at location {}'.format(self.dataset_name, self.data_path))

        self.file_path = os.path.abspath(
            os.path.join(self.data_path, self.url.split('/')[-1].split('.zip')[0] + '.ply'))
        return self.file_path

    def download_semantic_tree_dataset(self, output_path: str = os.path.abspath(__file__), overwrite: bool = False):

        self.url = 'https://faubox.rrze.uni-erlangen.de/dl/fiCVjuo7hpxcacu1dWEmr9/semantic_tree.zip'

        self.dataset_name = 'semantic tree'

        existence = self.__check_existence(output_directory=output_path, dataset_name=self.dataset_name)

        if existence == NON_EXIST:
            self.filename = download(url=self.url, output_dir=self.data_path, overwrite=overwrite)
            extract(filename=self.filename, output_dir=self.data_path)
        else:
            print('Dataset {} already exists at location {}'.format(self.dataset_name, self.data_path))

        self.file_paths = glob.glob(
            os.path.join(self.data_path, self.url.split('/')[-1].split('.zip')[0] + '/*' + '.ply'))
        return self.file_paths


if __name__ == '__main__':
    downloader = Dataset()
    file = downloader.download_tree_dataset()

    print('Saved at {}'.format(downloader.dataset_path))

    downloader = Dataset()
    file1, file2 = downloader.download_semantic_tree_dataset()

    print('Saved at {}'.format(downloader.dataset_path))
