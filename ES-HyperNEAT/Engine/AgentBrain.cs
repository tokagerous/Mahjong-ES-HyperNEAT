using System;
using System.Collections.Generic;
using System.Text;
using SharpNeatLib.NeuralNetwork;
using Engine;
using SharpNeatLib.NeatGenome;
using System.Xml;
using SharpNeatLib.CPPNs;

namespace Engine
{
    //Abstracts away neural networks and heterogeneity
    public class AgentBrain
    {
        //How many times should the network get activated. TODO make configurable
        private static int NET_ACTIVATION_STEPS = 2;
       // private bool EVOLVED_SUBSTRATE = true;

        //The "big" brain for heterogenous teams
        public INetwork brain;

        //Brains for homogenous teams
        public List<INetwork> brains; 

        //Homogenous team?
        public bool homogenous;
        public int numRobots;
        private List<Robot> robotListeners;
        private bool allActivated;
      
        private float[] teamInput;
        private bool[] activated;

        private SubstrateDescription substrateDescription;
        public INetwork genome;
        public NeatGenome ANN;

        private bool normalizeANNWeights;
        private bool adaptableANN;
        private bool modulatoryANN;

        public bool multipleBrains = false;
        public bool evolveSubstrate = false;

        public List<INetwork> multiBrains = new List<INetwork>();

        public List<float> zcoordinates;

        public AgentBrain(bool homogenous, int numAgents, SubstrateDescription substrateDescription, INetwork genome,
            bool normalizeANNWeights, bool adaptableANN, bool modulatoryANN, bool multi, bool evolveSubstrate)
        {
            this.evolveSubstrate = evolveSubstrate;
            this.normalizeANNWeights = normalizeANNWeights;
            this.adaptableANN = adaptableANN;
            this.modulatoryANN = modulatoryANN;
            this.genome = genome;
            this.substrateDescription = substrateDescription;
            this.numRobots = numAgents;
            this.homogenous = homogenous;
            this.multipleBrains = multi;

            //inputCounter = 0;
            teamInput = new float[numAgents * substrateDescription.InputCount];
            activated = new bool[numAgents];

            createBrains();
            
            robotListeners = new List<Robot>();
        }

        public void switchBrains()
        {
            if (multipleBrains)
                brain = multiBrains[1];
        }

        private void createBrains()
        {
            if (genome != null) 
            {
                if (homogenous)
                {

                    ANN = substrateDescription.generateHomogeneousGenome(genome, normalizeANNWeights, this.adaptableANN, this.modulatoryANN, evolveSubstrate);
                    brains = new List<INetwork>();
                    for (int i = 0; i < numRobots; i++)
                    {
                        INetwork b = ANN.Decode(null);
                        brains.Add(b);
                    }
                }
                else
                {
                    if (multipleBrains) //Multiple brains with situational policies
                    {
                        List<NeatGenome> genes = substrateDescription.generateGenomeStackSituationalPolicy(genome, Convert.ToUInt32(numRobots), normalizeANNWeights, adaptableANN, modulatoryANN, 2, out zcoordinates);

                        for (int j = 0; j < genes.Count; j++)
                            multiBrains.Add(genes[j].Decode(null));

                        brain = multiBrains[0];

                    }
                    else
                    {

                        ANN = substrateDescription.generateMultiGenomeStack(genome, Convert.ToUInt32(numRobots), normalizeANNWeights, adaptableANN,
                                                                        modulatoryANN, out zcoordinates, evolveSubstrate);
                        brain = ANN.Decode(null);
                    }
                }
           }
        }

        //Needs to be called if the number of sensors changes
        //Right now all brains have the same number of inputs
        public void updateInputDensity()
        {
            teamInput = new float[numRobots * substrateDescription.InputCount];
            //Check if the number of sensors changed so we have to regenerate the ANNs
            if (homogenous)
            {
                if (brains!=null && brains[0]!=null && brains[0].InputNeuronCount != substrateDescription.InputCount)
                {
                    Console.WriteLine("Recreating ANNs");
                    createBrains();
                }
            }
            else
            {
                if (brain!=null && (brain.InputNeuronCount/numRobots) != substrateDescription.InputCount)
                {
                    Console.WriteLine("Recreating ANNs");
                    createBrains();
                }

            }
        }

        public void clearRobotListeners()
        {
            robotListeners.Clear();
        }

        //Robots need to be registered before they can receive ANN results
        public void registerRobot(Robot robot)
        {
            if (robotListeners.Contains(robot))
            {
                Console.WriteLine("Robot " + robot.id + " already registered");
                return;
            }
            robotListeners.Add(robot);
            if (robotListeners.Count > numRobots)
            {
                Console.WriteLine("Number of registered agents ["+robotListeners.Count+"] and number of agents ["+numRobots+"] does not match");
            }
        }

        public INetwork getBrain(int number)
        {
            if (homogenous)
            {
                if (brains==null) return null;
                return brains[number];
            }
            else
                return brain;   //only one brain for heterogenous teams
        }

        //Clear the listener list
        public void reset()
        {
            robotListeners.Clear();
        }

        //call once all agenht received their inputs
        public void execute(System.Threading.Semaphore sem)
        {
            //return;
            if (homogenous)
            {
                float[] inputs = new float[substrateDescription.InputCount];

                for (int agentNumber=0; agentNumber<numRobots; agentNumber++) 
                {
                    //prepare inputs 
                    for (int i = 0; i < substrateDescription.InputCount; i++)
                    {
                        inputs[i] = teamInput[(i + agentNumber * substrateDescription.InputCount)];
                    }

                    brains[agentNumber].SetInputSignals(inputs);
                    brains[agentNumber].MultipleSteps(NET_ACTIVATION_STEPS); //maybe base this on ANN depth

                    float[] outputs = new float[brains[agentNumber].OutputNeuronCount];
                    for (int j = 0; j < outputs.Length; j++)
                    {
                        outputs[j] = brains[agentNumber].GetOutputSignal(j);
                    }

                    robotListeners[agentNumber].networkResults(outputs);              
                }
                return;
            }

            if (brain == null)
                return;

            //Heterogenous
            brain.SetInputSignals(teamInput);
            brain.MultipleSteps(NET_ACTIVATION_STEPS); 

			
            int out_count = 0;
            int numOutputAgent = brain.OutputNeuronCount / numRobots;
            float[] outp = new float[numOutputAgent];
			foreach (Robot robot in robotListeners)
            {
                for (int y = 0; y < numOutputAgent; y++)
                {
                    outp[out_count % numOutputAgent] = brain.GetOutputSignal(out_count);

                    if (Double.IsNaN(outp[out_count % numOutputAgent]))
                        Console.WriteLine("NaN in outputs");
                    out_count++;
                }
				
				
				//              Console.WriteLine("Sending results to " + robot.id);
                //sem.WaitOne();
				robot.networkResults(outp);
				//sem.Release();
            }
          //  Console.WriteLine(outp[0] + " " + outp[1] + " " + outp[2]);



				bool debug=false;
				if (debug) {
				//for(int id=0;id<numRobots;id++) {
				for(int id=1;id<=1;id++) {
					Console.Write(id+" INPUTS: ");
				for(int i=0;i<substrateDescription.InputCount;i++) 
					Console.Write(teamInput[i + id * substrateDescription.InputCount]+" ");
				Console.WriteLine();
				Console.Write(id+" OUTPUTS: ");
		        int biggest=0;
				float val=0.0f;
				for(int i=0;i<3;i++) //numOutputAgent;i++)
					{
						float temp = brain.GetOutputSignal(numOutputAgent*id+i);
						Console.Write(temp+ " ");
						if (temp>val)
						{
							biggest=i;
							val=temp;
						}
					}
					
				
				Console.Write(biggest); //brain.GetOutputSignal(numOutputAgent*id+i)+" ");
				
				Console.WriteLine();
				}
				}
	
			
        }

        //TODO not really clean
        public void clearANNSignals(float zstack)
        {
            int index = 0;
            ModularNetwork net = ((ModularNetwork)brain);
            foreach (ConnectionGene gene in net.genome.ConnectionGeneList)
            {
                if (gene.coordinates.Length > 4 && !gene.coordinates[4].Equals(float.NaN))
                {
                    if (gene.coordinates[4] != zstack)        //Only valid if robot has z-values
                    {
                        index++;
                        continue;
                    }
                }

                ((ModularNetwork)brain).neuronSignals[net.connections[index].targetNeuronIdx] = 0.0f;
                ((ModularNetwork)brain).neuronSignals[net.connections[index].sourceNeuronIdx] = 0.0f;
                    
                
            }
        }


        //Activate the agents network with the given inputs
        public void setInputSignals(int agentNumber, float[] inputs)
        {
            for (int i = 0; i < inputs.Length; i++)
            {
                teamInput[(i + agentNumber * inputs.Length)] = inputs[i];
            }
        }

        public void setInputSignalsOLD(int agentNumber, float[] inputs)
        {
            if (homogenous)
            {
                brains[agentNumber].SetInputSignals(inputs);
                brains[agentNumber].MultipleSteps(NET_ACTIVATION_STEPS); //maybe base this on ANN depth

                float[] outputs = new float[brains[agentNumber].OutputNeuronCount];
                for (int j = 0; j < outputs.Length; j++)
                {
                    outputs[j] = brains[agentNumber].GetOutputSignal(j);
                }

                robotListeners[agentNumber].networkResults(outputs);

                return;
            }

            if (brain == null)
                return;

            //  Console.WriteLine("Received :"+agentNumber+" input length "+inputs.Length+" inputcounter "+inputCounter);

            for (int i = 0; i < inputs.Length; i++)
            {
                //if (inputCounter != i+agentNumber * inputs.Length)
                //{
                //    Console.WriteLine(inputCounter + " " + (i + agentNumber * inputs.Length));
                //}

                teamInput[(i + agentNumber * inputs.Length)] = inputs[i];
                //inputCounter++;
            }

            activated[agentNumber] = true;

            allActivated = true;
            for (int i = 0; i < activated.Length; i++)
            {
                if (!activated[i])
                {
                    allActivated = false;
                    break;
                }
            }

            if (allActivated)       //weight for all ANN to get activated before sending out the networkResult event
            {
                //             Console.WriteLine("All agents activated");
                allActivated = false;
                //   inputCounter = 0;
                for (int i = 0; i < activated.Length; i++)
                {
                    activated[i] = false;
                }

                brain.SetInputSignals(teamInput);
                brain.MultipleSteps(NET_ACTIVATION_STEPS);

                int out_count = 0;
                int numOutputAgent = brain.OutputNeuronCount / numRobots;
                float[] outputs = new float[numOutputAgent];
                foreach (Robot robot in robotListeners)
                {
                    for (int y = 0; y < numOutputAgent; y++)
                    {
                        outputs[out_count % numOutputAgent] = brain.GetOutputSignal(out_count);

                        if (Double.IsNaN(outputs[out_count % numOutputAgent]))
                            Console.WriteLine("NaN in outputs");
                        out_count++;
                    }

                    //              Console.WriteLine("Sending results to " + robot.id);
                    robot.networkResults(outputs);
                }
            }
        }
    }
}