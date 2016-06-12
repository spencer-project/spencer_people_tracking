#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Timm Linder, Social Robotics Lab, University of Freiburg
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


from __future__ import print_function
import sys, os, numpy, re


COLOR_ERROR = '\033[91m'
COLOR_DEFAULT = '\033[98m'

def num(s):
    try:
        return int(s)
    except ValueError:
        return float(s)

def formatField(value, maxlen = 10):
    if type(value) == str:
        valueStr = value
    else:
        if numpy.isfinite(value) and (abs(int(value) - value) < 0.001 or value > 5):
            valueStr = "%d" % int(value)
        else:
            valueStr = "%.3f" % value

    return valueStr.ljust(max(len(valueStr)+1, maxlen))

def aggregateResults(resultsPath, numExpectedGTCycles):
    #print("Aggregating results in folder: " + resultsPath)

    if not resultsPath.endswith("/"):
        resultsPath += "/"

    if not os.path.isdir(resultsPath):
        sys.stderr.write(COLOR_ERROR + 'Specified results folder does not exist: %s\n' % resultsPath)
        return

    # Read unaggregated result files
    pymotResults = dict()
    timingStatistics = dict()
          
    for folder in next(os.walk(resultsPath))[1]:
        if not re.match("\d\d\d\d-\d\d-\d\d-\d\d-\d\d", folder):
            print("Skipping results folder which does not match naming criterion: " + folder)
            continue

        subDir = resultsPath + folder

        ignoreThisRun = False
        for filename in next(os.walk(subDir))[2]:
            filePath = subDir + "/" + filename

            # Read PyMot
            if filename.startswith("pymot_metrics") and filename.endswith(".txt"):
                with open(filePath) as f:
                    lines = f.readlines()

                if lines:
                    pymotKeys = lines[0].split()
                    pymotValues = lines[1].split()

                    for i in xrange(0, len(pymotKeys)):
                        pymotKey = pymotKeys[i]
                        pymotValue = num(pymotValues[i])

                        if pymotKey == "cycles_synched_with_gt":
                            if pymotValue < numExpectedGTCycles:
                                sys.stderr.write('### Ignoring pyMot results because there are less than %d synched groundtruth cycles: %s\n ###\n' % (numExpectedGTCycles, filePath) )
                                ignoreThisRun = True

                    if not ignoreThisRun:
                        for i in xrange(0, len(pymotKeys)):
                            pymotKey = pymotKeys[i]
                            pymotValue = num(pymotValues[i])

                            if not pymotKey in pymotResults:
                                pymotResults[pymotKey] = []

                            pymotResults[pymotKey].append(pymotValue)

                    # Recover relative number of ID switches from old pymot results files which did not yet include this field
                    if not ignoreThisRun and not "relative_id_switches" in pymotKeys:
                        if "misses" in pymotKeys and "fp" in pymotKeys and "mota" in pymotKeys and "mismatches" in pymotKeys:
                            misses = float(pymotValues[pymotKeys.index("misses")])
                            fp = float(pymotValues[pymotKeys.index("fp")])
                            mota = float(pymotValues[pymotKeys.index("mota")])
                            idSwitches = float(pymotValues[pymotKeys.index("mismatches")])

                            if (1.0 - mota) != 0.0:
                                gt = (misses + fp + idSwitches) / (1.0 - mota)

                                if (gt - misses) != 0.0:
                                    recoveredRelativeIdSwitches = idSwitches * gt / (gt - misses)
                                    if not "relative_id_switches" in pymotResults:
                                        pymotResults["relative_id_switches"] = []
                                    
                                    pymotResults["relative_id_switches"].append(recoveredRelativeIdSwitches)

            if ignoreThisRun:
                break

            # Read timing statistics
            if filename.startswith("timing_metrics") and filename.endswith(".txt"):
                with open(filePath) as f:
                    lines = f.readlines()

                if lines:
                    timingKeys = [key.strip() for key in lines[0].split(";")]

                    for key in timingKeys:
                        if not key in timingStatistics:
                            timingStatistics[key] = []

                    for row in xrange(1, len(lines)):
                        valuesInRow = lines[row].split(";")

                        for col in xrange(0, len(valuesInRow)):
                            key = timingKeys[col]
                            timingStatistics[key].append( num(valuesInRow[col]) )

    if pymotResults:
        # Aggregate PyMot results
        aggregatedPymotPath = resultsPath + "pymot_aggregated.txt"
        with open(aggregatedPymotPath, 'w') as f:
            print(formatField("", 30),  formatField("mean"), formatField("median"), formatField("std"), formatField(""), formatField("min"), formatField("max"), "raw_values", file=f)
            for pymotKey in sorted(pymotResults):
                pymotValues = pymotResults[pymotKey]
                mean = numpy.mean(numpy.array(pymotValues))
                median = numpy.median(numpy.array(pymotValues))
                std = numpy.std(numpy.array(pymotValues))
                minVal = numpy.min(numpy.array(pymotValues))
                maxVal = numpy.max(numpy.array(pymotValues))

                print(formatField(pymotKey, 30), formatField(mean), formatField(median), formatField(std), formatField(""), formatField(minVal), formatField(maxVal), pymotValues, file=f)

            print(COLOR_DEFAULT + "Aggregated pyMot results have been written to: " + aggregatedPymotPath)
    else:
        print(COLOR_DEFAULT + "No pyMot results found, not aggregating!")

    if timingStatistics:
        # Aggregate timing results
        aggregatedTimingStatisticsPath = resultsPath + "timing_aggregated.txt"
        with open(aggregatedTimingStatisticsPath, 'w') as f:
            print(formatField("", 30),  formatField("mean"), formatField("median"), formatField("std"), formatField(""), formatField("min"), formatField("max"), file=f)
            for timingKey in sorted(timingStatistics):
                timingValues = timingStatistics[timingKey]
                mean = numpy.mean(numpy.array(timingValues))
                median = numpy.median(numpy.array(timingValues))
                std = numpy.std(numpy.array(timingValues))
                minVal = numpy.min(numpy.array(timingValues))
                maxVal = numpy.max(numpy.array(timingValues))

                print(formatField(timingKey, 30), formatField(mean), formatField(median), formatField(std), formatField(""), formatField(minVal), formatField(maxVal), file=f)

            print(COLOR_DEFAULT + "Aggregated timing statistics have been written to: " + aggregatedTimingStatisticsPath)
    else:
        print(COLOR_DEFAULT + "No timing statistics results found, not aggregating!")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.stderr.write(COLOR_ERROR + 'Path to results folder not specified!\n')
        sys.exit(1)

    resultsPath = sys.argv[1]
    aggregateResults(resultsPath, 2400)