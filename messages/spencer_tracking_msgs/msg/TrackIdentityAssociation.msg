# Message defining a tracked person and its association with an identity(name)

uint64      track_id        # unique identifier of the target, consistent over time
uint64      detection_id    # id of the corresponding detection in the current cycle (undefined if occluded)
string      person_name	    # name of the person. This will be used in talk with the person