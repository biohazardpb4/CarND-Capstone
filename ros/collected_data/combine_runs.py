#!/bin/python

from os import walk, path
from shutil import copyfile

i = 0
to='.'
for (dirpath, dirnames, filenames) in walk('.'):
    for name in filenames:
        copyfile(path.join(dirpath, name), path.join(to, "{:04d}.jpg".format(i)))
        i+=1