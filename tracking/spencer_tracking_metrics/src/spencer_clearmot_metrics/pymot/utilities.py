import sys

def write_stderr_red(prefix, message):
    if sys.stderr.isatty():
        sys.stderr.write("\x1b[;31m%s \x1b[0m" % (prefix)) # Write red prefix
    else:
        sys.stderr.write(prefix + " ") # Write prefix

    sys.stderr.write(message + "\n")

