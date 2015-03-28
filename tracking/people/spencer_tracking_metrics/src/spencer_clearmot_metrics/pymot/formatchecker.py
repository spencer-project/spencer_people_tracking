from utilities import write_stderr_red

class FormatChecker:

    def __init__(self, groundtruth, hypotheses):
        """Constructor """

        self.groundtruth_ = groundtruth
        self.hypotheses_ = hypotheses


    def checkForAmbiguousIDs(self):
        """Check ground truth and hypotheses for multiple use of the same id per frame"""

        result = True
        
        for frame in self.groundtruth_["frames"]:
            ids = set()
            for groundtruth in frame["annotations"]:
                if not "id" in groundtruth:
                    # We should have already warned about a missing ID in checkForExistingIDs
                    # no need to raise an exception in this function by trying to access missing IDs
                    continue

                if groundtruth["id"] in ids:
                    result &= False
                    write_stderr_red("Warning:", "Ambiguous id (%s) found in ground truth, timestamp %f, frame %d!" % (str(groundtruth["id"]), frame["timestamp"], frame["num"] if "num" in frame else -1))
                else:
                    ids.add(groundtruth["id"])
        
        for frame in self.hypotheses_["frames"]:
            ids = set()
            for hypothesis in frame["hypotheses"]:
                if hypothesis["id"] in ids:
                    result &= False
                    write_stderr_red("Warning:", "Ambiguous hypothesis (%s) found in hypotheses, timestamp %f, frame %d!" % (str(hypothesis["id"]), frame["timestamp"], frame["num"] if "num" in frame else -1))
                else:
                    ids.add(hypothesis["id"])

        return result # true: OK, false: ambiguous id found


    def checkForExistingIDs(self):
        """Check ground truth and hypotheses for having a valid id. Valid: existing and non-empty."""

        result = True

        for f in self.groundtruth_["frames"]:
            for g in f["annotations"]:
                
                if not "id" in g.keys():
                    write_stderr_red("Warning:", "Groundtruth without ID found, timestamp %f, frame %d!" % (f["timestamp"], f["num"] if "num" in f else -1))
                    result &= False
                    continue

                if g["id"] == "":
                    write_stderr_red("Warning:", "Groundtruth with empty ID found, timestamp %f, frame %d!" % (f["timestamp"], f["num"] if "num" in f else -1))
                    result &= False
                    continue

        # Hypotheses without ids or with empty ids
        for f in self.hypotheses_["frames"]:
            for h in f["hypotheses"]:
                if not "id" in h.keys():
                    write_stderr_red("Warning:", "Hypothesis without ID found, timestamp %f, frame %d!" % (f["timestamp"], f["num"] if "num" in f else -1))
                    result &= False
                    continue

                if h["id"] == "":
                    write_stderr_red("Warning:", "Hypothesis with empty ID found, timestamp %f, frame %d!" % (f["timestamp"], f["num"] if "num" in f else -1))
                    result &= False
                    continue

        return result # true: OK, false: missing id found


    def checkForCompleteness(self):
        """Check ground truth and hypotheses for containing width, height, x and y"""

        result = True
        expectedKeys = ("x", "y", "width", "height")

        for f in self.groundtruth_["frames"]:
            for g in f["annotations"]:
                for key in expectedKeys:
                    if not key in g.keys():
                        write_stderr_red("Warning:", "Groundtruth without key %s found, timestamp %f, frame %d!" % (key, f["timestamp"], f["num"] if "num" in f else -1))
                        result &= False

        # Hypotheses without ids or with empty ids
        for f in self.hypotheses_["frames"]:
            for h in f["hypotheses"]:
                for key in expectedKeys:
                    if not key in h.keys():
                        write_stderr_red("Warning:", "Hypothesis without key %s found, timestamp %f, frame %d!" % (key, f["timestamp"], f["num"] if "num" in f else -1))
                        result &= False

        return result # true: OK, false: missing id found
