
void AG95_Test()
{
    DH_AG95_CAN m_gripper(1, "192.168.1.29:8888", 0);
    if(m_gripper.open()<0)
    {
        return ;
    }
    //initialize the gripper
    int initstate = 0;
    m_gripper.GetInitState(&initstate);
    if(initstate != DH_AG95_CAN::S_INIT_FINISHED)
    {
        m_gripper.Initialization();
        std::cout<< " Send grip init " << std::endl;

        //wait for gripper initialization
        initstate = 0;
        std::cout<< " Send grip GetInitState " << std::endl;
        while(initstate != DH_AG95_CAN::S_INIT_FINISHED )
            m_gripper.GetInitState(&initstate); 
        std::cout<< " Send grip GetInitState "<< initstate << std::endl;
    }

    int currpos= 0;
    int g_state = 0;
    int loop = 100;
    while(loop--)
    {

    //set gripper target position 100
        m_gripper.SetTargetPosition(100);
        
    //wait gripper arrived the target postion
        g_state = 0;
        while(g_state == DH_AG95_CAN::S_GRIP_MOVING)
            m_gripper.GetGripState(&g_state);
        std::cout<< "1 current grip state " << g_state << std::endl;

    //get gripper current position
        m_gripper.GetCurrentPosition(&currpos);
        std::cout<< "2 current posistion " << currpos << std::endl;

    //set gripper target position 0
        m_gripper.SetTargetPosition(0);
    
    //wait gripper catch a object or arrived target position
        g_state = 0;
        while(g_state == DH_DH3_CAN::S_GRIP_MOVING)
            m_gripper.GetGripState(&g_state);
        std::cout<< "3 current state " << g_state << std::endl;

    //get gripper current position
        m_gripper.GetCurrentPosition(&currpos);
        std::cout<< "6 current position " << currpos << std::endl;

        std::cout<<  std::endl;
        std::cout<<  std::endl;
     }
     m_gripper.close();
}

void ModbusGripper_test()
{
    DH_Modbus_Gripper m_gripper(1, "/dev/ttyUSB0", 115200);
    if(m_gripper.open()<0)
    {
        return ;
    }
    //initialize the gripper
    int initstate = 0;
    m_gripper.GetInitState(&initstate);
    if(initstate != DH_Modbus_Gripper::S_INIT_FINISHED)
    {
        m_gripper.Initialization();
        std::cout<< " Send grip init " << std::endl;

        //wait for gripper initialization
        initstate = 0;
        std::cout<< " Send grip GetInitState " << std::endl;
        while(initstate != DH_Modbus_Gripper::S_INIT_FINISHED )
            m_gripper.GetInitState(&initstate); 
        std::cout<< " Send grip GetInitState "<< initstate << std::endl;
    }

    int currpos= 0;
    int g_state = 0;
    int loop = 100;
    while(loop--)
    {

    //set gripper target position 1000
        m_gripper.SetTargetPosition(1000);
        
    //wait gripper arrived the target postion
        g_state = 0;
        while(g_state == DH_Modbus_Gripper::S_GRIP_MOVING)
            m_gripper.GetGripState(&g_state);
        std::cout<< "1 current grip state " << g_state << std::endl;

    //get gripper current position
        m_gripper.GetCurrentPosition(&currpos);
        std::cout<< "2 current posistion " << currpos << std::endl;

    //set gripper target position 0
        m_gripper.SetTargetPosition(0);
    
    //wait gripper catch a object or arrived target position
        g_state = 0;
        while(g_state == DH_Modbus_Gripper::S_GRIP_MOVING)
            m_gripper.GetGripState(&g_state);
        std::cout<< "3 current state " << g_state << std::endl;

    //get gripper current position
        m_gripper.GetCurrentPosition(&currpos);
        std::cout<< "6 current position " << currpos << std::endl;

        std::cout<<  std::endl;
        std::cout<<  std::endl;
     }
     m_gripper.close();
}

void DH3_Test()
{
    DH_DH3_CAN m_gripper(1, "192.168.1.29:8888", 0);
    if(m_gripper.open()<0)
    {
        return ;
    }
    //initialize the gripper
    int initstate = 0;
    m_gripper.GetInitState(&initstate);
    if(initstate != DH_DH3_CAN::S_INIT_FINISHED)
    {
            m_gripper.Initialization();
    std::cout<< " Send grip init " << std::endl;

    //wait for gripper initialization
    initstate = 0;
     std::cout<< " Send grip GetInitState " << std::endl;
    while(initstate != DH_DH3_CAN::S_INIT_FINISHED )
        m_gripper.GetInitState(&initstate); 
         std::cout<< " Send grip GetInitState "<< initstate << std::endl;
    }

    int currpos= 0;
    int g_state = 0;
    int curangle = 0;
    int r_state = 0;
    int loop = 100;
    int testrotation = 100;
    while(loop--)
    {

    //set gripper target position 95
        m_gripper.SetTargetPosition(95);
        
    //wait gripper arrived the target postion
        g_state = 0;
        while(g_state == DH_DH3_CAN::S_GRIP_MOVING)
            m_gripper.GetGripState(&g_state);
        std::cout<< "1 current grip state " << g_state << std::endl;
        
        // sleep(1);

    //get gripper current position
        m_gripper.GetCurrentPosition(&currpos);
        std::cout<< "2 current posistion " << currpos << std::endl;


    // //set gripper target rotation angle 0 degree
        m_gripper.SetTargetRotation(testrotation);
    
    // //wait gripper arrived the target angle
        r_state = 0;
        while(r_state == DH_DH3_CAN::S_ROT_MOVING)
            m_gripper.GetRotationState(&r_state);
        std::cout<< "3 current rotation state " << r_state << std::endl;
         
        // sleep(1);

    // //get gripper current angle
        m_gripper.GetCurrentRotation(&curangle);
        std::cout<< "4 current angle " << curangle << std::endl;

    //set gripper target position 0
        m_gripper.SetTargetPosition(0);
    
     //wait gripper catch a object or arrived target position
        g_state = 0;
        while(g_state == DH_DH3_CAN::S_GRIP_MOVING)
            m_gripper.GetGripState(&g_state);
        std::cout<< "5 current state " << g_state << std::endl;

        // sleep(1);

    //get gripper current position
        m_gripper.GetCurrentPosition(&currpos);
        std::cout<< "6 current position " << currpos << std::endl;


        m_gripper.GetRotationState(&r_state);
        std::cout<< "7 current rotation state " << r_state << std::endl;
    
    //     // sleep(1);
        testrotation = testrotation -10 ;
        if(testrotation<0)
            testrotation = 100;

        std::cout<<  std::endl;
        std::cout<<  std::endl;

     }
     m_gripper.close();
}