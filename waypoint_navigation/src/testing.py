from waypoint_navigation import Map


print "Map being used by the robot:"
test_map = Map((-10,-10), (20,20), 1)

print "to grid"
print "centre should return (10,10)"
print test_map.to_grid( (0, 0) ) # centre

print "lower bounds should return (0, 0)"
print test_map.to_grid( (-10, -10) ) # in bounds
print "out of bounds should return None"
print test_map.to_grid( (-11, -11) ) # out of bounds

print "upper bounds should return (19,19)"
print test_map.to_grid( (9.9999, 9.9999) ) # in bounds
print "out of bounds should return None"
print test_map.to_grid( (10, 10) ) # out of bounds

print "\nto world"
print "centre should return (0.5, 0.5)"
print test_map.to_world( (10, 10) ) # centre

print "lower bounds should return (-9.5, -9.5)"
print test_map.to_world( (0, 0) ) # in bounds
print "out of bounds should return None"
print test_map.to_world( (-1, -1) ) # out of bounds

print "upper bounds should return (9.5, 9.5)"
print test_map.to_world( (19, 19) ) # in bounds
print "out of bounds should return None"
print test_map.to_world( (20, 20) ) # out of bounds


print "\n0.5 resolution map"
test_map = Map((-10,-10), (20,20), 0.5)

print "to grid"
print "centre should return (10,10)"
print test_map.to_grid( (-5, -5) ) # centre

print "lower bounds should return (0, 0)"
print test_map.to_grid( (-10, -10) ) # in bounds
print "out of bounds should return None"
print test_map.to_grid( (-11, -11) ) # out of bounds

print "upper bounds should return (19, 19)"
print test_map.to_grid( (-0.001, -0.001) ) # in bounds
print "out of bounds should return None"
print test_map.to_grid( (0, 0) ) # out of bounds

print "\nto world"
print "centre should return (-4.75, -4.75)"
print test_map.to_world( (10, 10) ) # centre

print "lower bounds should return (-9.75, -9.75)"
print test_map.to_world( (0, 0) ) # in bounds
print "out of bounds should return None"
print test_map.to_world( (-1, -1) ) # out of bounds

print "upper bounds should return (-0.25, -0.25)"
print test_map.to_world( (19, 19) ) # in bounds
print "out of bounds should return None"
print test_map.to_world( (20, 20) ) # out of bounds

print "2 resolution map:"
test_map = Map((-10,-10), (20,20), 2)

print "to grid"
print "centre should return (10,10)"
print test_map.to_grid( (10, 10) ) # centre

print "lower bounds should return (0, 0)"
print test_map.to_grid( (-10, -10) ) # in bounds
print "out of bounds should return None"
print test_map.to_grid( (-11, -11) ) # out of bounds

print "upper bounds should return (19, 19)"
print test_map.to_grid( (29.999, 29.999) ) # in bounds
print "out of bounds should return None"
print test_map.to_grid( (30, 30) ) # out of bounds

print "\nto world"
print "centre should return (11, 11)"
print test_map.to_world( (10, 10) ) # centre

print "lower bounds should return (-9, -9)"
print test_map.to_world( (0, 0) ) # in bounds
print "out of bounds should return None"
print test_map.to_world( (-1, -1) ) # out of bounds

print "upper bounds should return (29, 29)"
print test_map.to_world( (19, 19) ) # in bounds
print "out of bounds should return None"
print test_map.to_world( (20, 20) ) # out of bounds

print "\nrectangular map bound testing"
test_map = Map((0,0),(5,10), 1)
print "in bounds"
print test_map.to_grid( (4, 9) )
print test_map.to_world( (4, 9) )
print "out of bounds should all return None"
print test_map.to_grid((5, 9))
print test_map.to_grid((4, 10))
print test_map.to_world((5, 9))
print test_map.to_world((4, 10))

test_map = Map((0,0), (10,5), 1)
print "in bounds"
print test_map.to_grid( (9, 4) )
print test_map.to_world( (9, 4) )
print "out of bounds should all return None"
print test_map.to_grid((9, 5))
print test_map.to_grid((10, 4))
print test_map.to_world((9, 5))
print test_map.to_world((10, 4))

print "module wiki test cases"
test_map = Map((0,0), (20,20), 1)
print "should return (5,5)"
print test_map.to_grid( (5, 5) ) 
test_map = Map((0,0), (20,20), 0.5)
print "should return (10,10)"
print test_map.to_grid( (5, 5) )
test_map = Map((-5, -5), (20, 20), 1)
print "should return (5,5)"
print test_map.to_grid( (0,0) )
test_map = Map((-5,-5), (20,20), 1)
print "should return (4.5, 4.5)"
print test_map.to_world( (9,9) ) 