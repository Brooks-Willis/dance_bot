import st 

arm = st.StArm()
arm.start()
arm.calibrate()
arm.home()

arm.continuous()
arm.learn("TEST1",[[-3000,0,5500],[0,3000,5500],[3000,0,5500]])

arm.set_point("P1")

arm.run_route("TEST1")