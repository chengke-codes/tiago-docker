find_package(rostest REQUIRED)

add_rostest(upload.test ARGS robot:=box)
add_rostest(upload.test ARGS robot:=four_link_chain)
add_rostest(upload.test ARGS robot:=four_link_star)
add_rostest(upload.test ARGS robot:=pendulum)
add_rostest(upload.test ARGS robot:=sphere)
add_rostest(upload.test ARGS robot:=simple_pmb2)
