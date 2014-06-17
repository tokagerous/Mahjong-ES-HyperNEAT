using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;
using SharpNeatLib;

namespace Engine
{
	public class CoordinateFrame {
			public float cx,cy;
			public float scale;
			public float rotation;
			public CoordinateFrame(float _dx,float _dy,float _scale,float _rotation) {
				cx=_dx;
				cy=_dy;
				scale=_scale;
				rotation=_rotation;
			}
			public void sync_from_environment(Environment x) {
				cx=x.view_x;
				cy=x.view_y;
				scale=x.view_scale;
			}
			public void sync_to_environment(Environment x) {
				x.view_x=cx;
				x.view_y=cy;
				x.view_scale=scale;
			}
		    public Point to_display(float x,float y) {
			 float ox,oy;
			  to_display(x,y,out ox,out oy);
			 return new Point((int)ox,(int)oy);
			}
		
			public Point to_display(Point x) {
				float ox,oy;
				return to_display(x.X,x.Y);
			}
		
			public Point2D to_display(Point2D p) {
				float ox,oy;
				to_display((float)p.x,(float)p.y,out ox, out oy);
				return new Point2D((double)ox,(double)oy);
			}
		
			//changes an input coordinate in simulator space to display space
			public void to_display(float ix,float iy,out float ox, out float oy) 
			{
				ox = (ix-cx)/scale;
			    oy = (iy-cy)/scale;
			}
		
			//changes an input coordinate in display space to simulator space
			public void from_display(float ix,float iy,out float ox, out float oy) 
			{
				ox = ix*scale+cx;
			    oy = iy*scale+cy;
			}
			
			public void offset_to_display(float ix, float iy, out float ox, out float oy)
			{
				ox = ix/scale;
				oy = iy/scale;
			}
		
			//input is a delta, not an absolute point in terms of screen coordinates
			//output is a delta in terms of simulator coordinates
			public void offset_from_display(float ix, float iy, out float ox, out float oy)
			{
				ox = ix*scale;
				oy = iy*scale;
			}
	}
	
	public abstract class CollisionManager {
		public abstract CollisionManager copy();
		public abstract void Initialize(Environment e,SimulatorExperiment _exp,List<Robot> rbts);
		public virtual void SimulationStepCallback() { }
		public abstract bool RobotCollide(Robot r);
		public abstract double Raycast(double angle, double max_range, Point2D point, Robot owner,out SimulatorObject hit);
		public bool agentVisible;
		public bool agentCollide;
	}

	
	
    public class EngineUtilities
    {
		
        //TODO: Maybe put this elsewhere so we can set the seed
		//TODO: One static rng will conflict if we ever want to reimplement multithreading
        public static Random random = new Random();

        #region Pen Colors
        public static Pen RedPen = new Pen(System.Drawing.Brushes.Red, 2.0f);
        public static Pen BluePen = new Pen(System.Drawing.Brushes.Blue, 2.0f);
        public static Pen GreendPen = new Pen(System.Drawing.Brushes.Green, 2.0f);
        public static Pen YellowPen = new Pen(System.Drawing.Brushes.Yellow, 2.0f);
        public static Pen DashedPen = new Pen(Brushes.Black, 1.0f);

        public static SolidBrush backGroundColorBrush = new SolidBrush( Color.White ); 
        #endregion

        //TODO dashedPen.DashStyle = System.Drawing.Drawing2D.DashStyle.Dash;
      
        //TODO how can a new object handle collsions differently?
        #region Collision Handling
	
		

        //wall-wall (should never collide)
        public static bool collide(Wall a, Wall b)
        {
            return false;
        }

        public static bool collide(Wall wall, Robot robot)
        {
            Point2D a1 = new Point2D(wall.line.p1);
            Point2D a2 = new Point2D(wall.line.p2);
            Point2D b = new Point2D(robot.location.x, robot.location.y);
            if (!wall.visible)
                return false;
            double rad = robot.radius;
            double r = ((b.x - a1.x) * (a2.x - a1.x) + (b.y - a1.y) * (a2.y - a1.y)) / wall.line.length_sq();
            double px = a1.x + r * (a2.x - a1.x);
            double py = a1.y + r * (a2.y - a1.y);
            Point2D np = new Point2D(px, py);
            double rad_sq = rad * rad;

            if (r >= 0.0f && r <= 1.0f)
            {
                if (np.distance_sq(b) < rad_sq)
                    return true;
                else
                    return false;
            }

            double d1 = b.distance_sq(a1);
            double d2 = b.distance_sq(a2);
            if (d1 < rad_sq || d2 < rad_sq)
                return true;
            else
                return false;
        }

        public static bool collide(Robot a, Wall b)
        {
            return EngineUtilities.collide(b, a);
        }

        public static bool collide(Robot a, Robot b)
        {
            return a.circle.collide(b.circle);
        }

        #endregion

        //TODO: port to radar sensor class
        public static double scanCone(Radar rf, List<SimulatorObject> objList)
        {
            double distance = rf.max_range;
            //hitRobot = false;
            double new_distance;
			double heading=rf.owner.heading;
			Point2D point=new Point2D(rf.owner.location.x+rf.offsetx,rf.owner.location.y+rf.offsety);
			
            double startAngle = rf.startAngle + heading;
            double endAngle = rf.endAngle + heading;
            double twoPi = 2 * Math.PI;

            if (startAngle < 0)
            {
                startAngle += twoPi;
            }
            else if (startAngle > twoPi)
            {
                startAngle -= twoPi;
            }
            if (endAngle < 0)
            {
                endAngle += twoPi;
            }
            else if (endAngle > twoPi)
            {
                endAngle -= twoPi;
            }

           // if (agentsVisible)
                foreach (SimulatorObject obj in objList)
                {
                    bool found = false;

                    if (obj == rf.owner)
                        continue;

                    new_distance = point.distance(obj.location);
                     
                  //  if (new_distance - obj.radius <= rf.max_range) //TODO do we need this
                    //{
                      //TODO  before: double angle = Math.Atan2(robot2.circle.p.y - point.y, robot2.circle.p.x - point.x);
                         double angle = Math.Atan2(obj.location.y - point.y, obj.location.x - point.x);
                       
                        if (angle < 0)
                        {
                            angle += Utilities.twoPi;
                        }

                        if (endAngle < startAngle)//sensor spans the 0 line
                        {
                            if ((angle >= startAngle && angle <= Math.PI * 2) || (angle >= 0 && angle <= endAngle))
                            {
                                found = true;
                            }
                        }
                        else if ((angle >= startAngle && angle <= endAngle))
                            found = true;
                   // }


                    if (found)
                    {
                        if (new_distance < distance)
                        {
                            distance = new_distance;
                       //     hitRobot = true;
                        }
                    }
                }

            return distance;

        }


        public static double euclideanDistance(Point p1, Point p2)
        {
            return Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
        }

        public static double euclideanDistance(Point2D p1, Point2D p2)
        {
            return Math.Sqrt(Math.Pow(p1.x - p2.x, 2) + Math.Pow(p1.y - p2.y, 2));
        }

        public static double euclideanDistance(Robot r1, Robot r2)
        {
            return Math.Sqrt(Math.Pow(r1.location.x - r2.location.x, 2) + Math.Pow(r1.location.y - r2.location.y, 2));
        }

        public static double squaredDistance(Robot r1, Robot r2)
        {
            return Math.Pow(r1.location.x - r2.location.x, 2) + Math.Pow(r1.location.y - r2.location.y, 2);
        }

        public static double clamp(double val, double min, double max)
        {
            if (val > max)
                val = max;
            else if (val < min)
                val = min;
            
            return val;
        }
    }
}
