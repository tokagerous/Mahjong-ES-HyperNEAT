using System;
using System.Collections.Generic;
using System.Text;

namespace Engine
{
    class Khepera3RobotModelComeHome : Khepera3RobotModel
    {
        public Khepera3RobotModelComeHome()
            : base()
        {
            name = "Khepera3RobotModelComeHome";
        }
    
        public override void populateSensors()
        {
            base.populateSensors();
            if (!agentBrain.multipleBrains)
                sensors.Add(new SignalSensor(this));
        }
    }
}
