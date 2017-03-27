from export_to_qgc import export_to_qgc

export = export_to_qgc('waypoints.txt')

export.takeoff(5,6,7)

wpt_list = []
wpt_list.append([1,2,3])
wpt_list.append([4,5,6])
wpt_list.append([7,8,9])

export.write_list2D(wpt_list)
