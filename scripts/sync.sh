#!/bin/bash
rsync -avz --exclude-from='exclude.txt' ~/humanoid $1:
