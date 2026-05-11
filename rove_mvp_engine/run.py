#!/usr/bin/env python3
"""Entry-point: `python run.py [options]`.

Thin shim around `engine.server.main()` so users don't have to remember
the package path."""

from engine.server import main


if __name__ == "__main__":
    main()
