#
# ~/.bashrc
#

# Alias
alias ls='ls --color=auto'
alias grep='grep --color=auto'
alias py=python3
alias c=clear

# Set colors
# These only apply when you use a terminal emulator inside the container
RED='\[\e[0;31m\]'
GREEN='\[\e[0;32m\]'
YELLOW='\[\e[0;33m\]'
BLUE='\[\e[0;34m\]'
PURPLE='\[\e[0;35m\]'
CYAN='\[\e[0;36m\]'
NC='\[\e[0m\]' # No Color

# PS
PS1="\[${GREEN}\t ${CYAN}\u${NC}@${YELLOW}\h${NC} ${BLUE}\w${NC} ${GREEN}\$${NC} \]"

# Workspace setup
source /opt/ros/kilted/setup.bash
