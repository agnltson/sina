# Pipeline

raw_data --(projection/snapping)--> data --(Walls: deleting dup corners)--> room_graph --(cut doors & clip bboxes)--> geometry --(add constraint)--> cdt
                                     |                                                  ^
                                     |-------------------------(project BBoxes)---------|
