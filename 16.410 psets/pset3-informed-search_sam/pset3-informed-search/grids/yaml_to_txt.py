#!/usr/bin/env python

import yaml
import sys
import numpy as np

if __name__ == '__main__':
    if len(sys.argv)<2:
        exit(1)
    f = open(sys.argv[1])
    data = yaml.safe_load(f.read())
    grid = np.array(data['GridPathPlanning']['GridData']['grid']).transpose().tolist()
    grid.reverse() # Reverse y order
    text_lines = map(lambda line: " ".join(map(lambda v: "1" if v!=0 else "0", line)), grid)
    grid_text = "\n".join(text_lines)
    print grid_text
        
    
    
