"""Double click for the desktop player, or pass CLI arguments."""
import sys
from dtwin.cli import main
if __name__=='__main__':raise SystemExit(main(sys.argv[1:] or ['gui']))
