from waypoint_navigation import Map

test_map = Map((-10,-10), (20,20), 1)

print test_map.to_grid( (0, 0) )
print test_map.to_grid( (-10, -10) )
print test_map.to_grid( (-11, -11) )
print test_map.to_grid( (9.9999, 9.9999) )
print "\n"
print test_map.to_world( (10, 10) )
print test_map.to_world( (0, 0) )
print test_map.to_world( (19, 19) )
print test_map.to_world( (-1, -1) )
print test_map.to_world( (20, 20) )
print "\n"

test_map = Map((-10,-10), (20,20), 0.5)

print test_map.to_grid( (-10, -10) )
print test_map.to_grid( (-9.5, -9.5) )
print test_map.to_grid( (-9, -9) )
print test_map.to_grid( (-0.001, -0.001) )
print test_map.to_grid( (-11, -11) )
print test_map.to_grid( (1, 1) )
print "\n"
print test_map.to_world( (10, 10) )
print test_map.to_world( (0, 0) )
print test_map.to_world( (19, 19) )
print test_map.to_world( (20, 20) )
print test_map.to_world( (-1, -1) )
