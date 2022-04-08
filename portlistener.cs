using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System;
//using System.Numerics;

[RequireComponent(typeof(LineRenderer))]


public class portlistener : MonoBehaviour
{
    public GameObject[] squareArray;
    public GameObject[] menuArray;
    public GameObject PistonOneEnd;
    public GameObject boomoneanimation;
    public LineRenderer Line;
    public LineRenderer PistonLine;
    public LineRenderer BoomOneCurve;
    public GameObject[] LinkArray;
   

    
    Thread mThread;
    public string connectionIP = "127.0.0.1";
    public int connectionPort = 25001;
    IPAddress localAdd;
    TcpListener listener;
    TcpClient client;
    public Vector3 receivedPos5 = new Vector3(-100,0,0);
    public Vector3 receivedPos6 = new Vector3(-100,0,0);
    public Vector3 receivedPos7 = new Vector3(-100,0,0);
    Vector3 size = Vector3.zero;
    Vector3 zeros = Vector3.zero;
    Vector3 outofframe = new Vector3(-100,0,0);
    public Vector3 MenuData = new Vector3(10,0,0);
    public Vector3 LinkMenuData = new Vector3(-10,0,0);
    //Vector3 LinkData = new Vector3(10,0,0);
    


    public bool running; //nothing

    public void Start() //private
    {
        
        ThreadStart ts = new ThreadStart(GetInfo);
        mThread = new Thread(ts);
        mThread.Start();
        //if(MenuData[0]==1){
        //StartCoroutine(FollowArrayPath());
        //}
        // float PistonFractionOne = 0.5f;
        // Vector3[] BoomOnePos = Calculations(receivedPos3, receivedPos4, receivedPos5, PistonFractionOne);
        // BoomOneCurve.positionCount = BoomOnePos.Length;
        
    }
    
    public void GetInfo() //nothing
    {
        localAdd = IPAddress.Parse(connectionIP);
        listener = new TcpListener(IPAddress.Any, connectionPort);
        listener.Start();

        client = listener.AcceptTcpClient();

        running = true;
        while (running)
        {
            SendAndReceiveData();
        }
        listener.Stop();
    }

    public void SendAndReceiveData() //nothing
    {
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];

        //---receiving Data from the Host----
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize); //Getting data in Bytes from Python
        string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead); //Converting byte data to string

        if (dataReceived != null)
        {
            //---Using received data---
            MenuData = StringMenu(dataReceived);
            LinkMenuData = StringLinkMenu(dataReceived);
            if(MenuData[0]==0){
            receivedPos5 = StringToVector3(dataReceived,"5");
            receivedPos6 = StringToVector3(dataReceived,"6");
            receivedPos7 = StringToVector3(dataReceived,"7");
            }


            
            

            //---Sending Data to Host----
            byte[] myWriteBuffer = Encoding.ASCII.GetBytes("Hey I got your message Python! Do You see this massage?"); //Converting string to byte data
            nwStream.Write(myWriteBuffer, 0, myWriteBuffer.Length); //Sending the data in Bytes to Python
        }
    }
  

    public static Vector3 StringToVector3(string sVector, string whichSquare)
    {
        // Remove the parentheses
        if (sVector.StartsWith("(") && sVector.EndsWith(")"))
        {
            sVector = sVector.Substring(1, sVector.Length - 2);
        }

        // split the items
        string[] sArray = sVector.Split(',');
        //get index of whichSquare
        int index = Array.IndexOf(sArray, whichSquare); 
        // store as a Vector3
        Vector3 result = new Vector3(
            float.Parse(sArray[index+1]),
            float.Parse(sArray[index+2]),
            float.Parse(sArray[index+3]));

        return result;
    }
    public static Vector3 StringMenu(string sVector)
    {
        // Remove the parentheses
        if (sVector.StartsWith("(") && sVector.EndsWith(")"))
        {
            sVector = sVector.Substring(1, sVector.Length - 2);
        }

        // split the items
        string[] sArray = sVector.Split(',');
        //get index of 24
        int index = Array.IndexOf(sArray, "24"); 
        Vector3 result = new Vector3(
            float.Parse(sArray[index+1]),0,0);
        return result;
    }
    public static Vector3 StringLinkMenu(string sVector)
    {
        // Remove the parentheses
        if (sVector.StartsWith("(") && sVector.EndsWith(")"))
        {
            sVector = sVector.Substring(1, sVector.Length - 2);
        }

        // split the items
        string[] sArray = sVector.Split(',');
        //get index of 24
        int index = Array.IndexOf(sArray, "23"); 
        Vector3 result = new Vector3(
            float.Parse(sArray[index+1]),0,0);
        return result;
    }
   
    // // George's Calculation here

    public float DistanceBetweenPoints(Vector3 v1, Vector3 v2)
        {
            float xDiff = (float)Math.Pow((v2[0] - v1[0]),2.0);
            float yDiff = (float)Math.Pow((v2[1] - v1[1]),2.0);

            return (float)Math.Sqrt(xDiff + yDiff);
        }
 

    
    public Vector3 PistonEnd(Vector3 BoomStart, Vector3 BoomEnd, float PistonFraction)
    {
            float BoomStartX = BoomStart[0];
            float BoomStartY = BoomStart[1]; 
            float BoomEndX = BoomEnd[0];
            float BoomEndY = BoomEnd[1];
            float PistonEndX = (BoomEndX-BoomStartX)*PistonFraction +BoomStartX;
            float PistonEndY = (BoomEndY-BoomStartY)*PistonFraction +BoomStartY; 

            Vector3 PistonEnd = new Vector3(PistonEndX, PistonEndY, 0f);
            return PistonEnd;

    }     

    

   
    public Vector3[] BoomRotationTest(Vector3 BoomFixed, Vector3 BoomEnd, Vector3 PistonStart, float BoomOverShootFraction, float PistonFraction, float TimeStep)
    {
        float BoomFixedX = BoomFixed[0];
        float BoomFixedY = BoomFixed[1]; 
        float BoomEndX = BoomEnd[0];
        float BoomEndY = BoomEnd[1];
                
        Vector3 VectorBoomEndToBoomFixed = BoomFixed - BoomEnd;
        Vector3 BoomStart = BoomFixed + (BoomOverShootFraction*VectorBoomEndToBoomFixed);
        float BoomStartX = BoomStart[0];
        float BoomStartY = BoomStart[1];
        float PistonEndX = (BoomEndX-BoomStartX)*PistonFraction +BoomStartX;
        float PistonEndY = (BoomEndY-BoomStartY)*PistonFraction +BoomStartY; 

        Vector3 PistonEnd = new Vector3(PistonEndX, PistonEndY, 0); 
        
         //Data from inputs
        float BoomLength = DistanceBetweenPoints(BoomStart, BoomEnd);
        float BoomStartToPistonLink = BoomLength*PistonFraction;
        float BoomFixedToPistonStart = DistanceBetweenPoints(BoomFixed, PistonStart);

        Vector3 BoomFinish = new Vector3(BoomEndX, BoomEndY, 0);
        //  if (BoomStartToPistonLink > BoomLength*(BoomOverShootFraction/(BoomOverShootFraction+1)))
        //  {
        //     Vector3 BoomFinish = new Vector3(BoomEndX, BoomEndY, 0);
        //  }
        if (BoomStartToPistonLink < BoomLength*(BoomOverShootFraction/(BoomOverShootFraction+1)))
        {
           BoomFinish[0] = BoomStartX;
           BoomFinish[1] = BoomStartY;
        } 
        float BoomFinishX = BoomFinish[0];
        float BoomFinishY = BoomFinish[1];

         //Requied for solve
        float BoomFixedToFinished = DistanceBetweenPoints(BoomFixed, BoomFinish);
        float BoomFixedToPistonLink = DistanceBetweenPoints(BoomFixed, PistonEnd);
        float PistonFractionAlongBoomFixedToFinish = BoomFixedToPistonLink/BoomFixedToFinished;

        //Piston Data
        float PistonRatio = 1.8f;
        float PistonRate = 0.5f;
        //bool Max = false; //Assume piston placed in minimum position unless defined as maximum
        float PistonMin = DistanceBetweenPoints(PistonStart, PistonEnd);
        float PistonMax = 0f;
        // if (Max = false)
        //{
        if ((PistonMin*PistonRatio)<(BoomFixedToPistonStart+BoomStartToPistonLink))
        {
        PistonMax = PistonMin*PistonRatio;
        }   
        else
        {
        PistonMax = BoomFixedToPistonStart+BoomStartToPistonLink; 
        }              
         //Time Data
        float PistonStep = PistonRate*TimeStep;
        int ArrayLength = (int)Math.Round((PistonMax-PistonMin)/PistonStep);
         
        //Piston Length Array 
        float[] PistonLength = new float[ArrayLength];
            for (int i = 0; i<3; i++)
            {
            PistonLength[i] = (float)( PistonMin + i*PistonStep );
            }
        //Time Array 
        float[] Time = new float[ArrayLength];
           for (int i = 0; i<ArrayLength; i++)
           {
            Time[i] = (float)(i*TimeStep);
           }
        //Solve 
        //z array
        float[] z =  new float[ArrayLength];
        for (int i = 0; i<ArrayLength; i++)
           {
               z[i] = ( (((float)Math.Pow(BoomFixedToPistonLink,2.0) + 
               (float)Math.Pow(BoomFixedToPistonStart,2.0) - 
               (float)Math.Pow(PistonLength[i],2.0))/(2*BoomFixedToPistonLink*BoomFixedToPistonStart))); 
           }
        //Theta - angle between Boom and vector BoomFixed to PistonStart - Alpha is the polar angle of 
        float Alpha = (float)Math.Atan2( (PistonEndY-BoomFixedY), (PistonEndX-BoomFixedX));
        float AlphaThetaOne = (float)Math.Atan2( (BoomFinishY-BoomFixedY), (BoomFinishX -BoomFixedX));
        //Orientation of Alpha and Theta
        int o1 = (int)Math.Round(AlphaThetaOne - Alpha - (float)Math.Acos(z[1]));
        int o2 = (int)Math.Round(AlphaThetaOne - Alpha + (float)Math.Acos(z[1]));
        int o3 = (int)Math.Round(AlphaThetaOne + Alpha - (float)Math.Acos(z[1]));
        int o4 = (int)Math.Round(AlphaThetaOne + Alpha + (float)Math.Acos(z[1]));

        float[] Theta = new float[ArrayLength];
        if (o1 == 0 || o1 == 360) 
        {
           for (int i = 0; i<ArrayLength; i++)
           {
               Theta[i]=( (float)Math.Acos(z[i]) );
           }
        }
        if (o2 == 0 || o2 == 360) 
        {
           for (int i = 0; i<ArrayLength; i++)
           {
               Theta[i]=( -(float)Math.Acos(z[i]) );
           }
        }
        if (o3 == 0 || o3 == 360) 
        {
           for (int i = 0; i<ArrayLength; i++)
           {
               Theta[i]=( (float)Math.Acos(z[i]) );
           }
           Alpha = -Alpha;
        }
        if (o4 == 0 || o4 == 360) 
        {
           for (int i = 0; i<ArrayLength; i++)
           {
               Theta[i]=( - (float)Math.Acos(z[i]) );
           }
           Alpha = -Alpha;
        }
        List<Vector3> PistonArrayList = new List<Vector3>();
		   for ( int i = 0; i < ArrayLength; i++)
		   {
               float x = BoomFixedToPistonLink*(float)Math.Cos(Theta[i]+Alpha);
               float y = BoomFixedToPistonLink*(float)Math.Sin(Theta[i]+Alpha);
		   	PistonArrayList.Add( new Vector3(x, y, 0f) );
		   }
		Vector3[] PistonArray = PistonArrayList.ToArray(); 
         List<Vector3> BoomArrayList = new List<Vector3>();
		   for ( int i = 0; i < ArrayLength; i++)
		   {
               float x = (BoomFixedX - PistonArray[i].x)/PistonFractionAlongBoomFixedToFinish + BoomFixedX; //idt you can call using [] like that, i get what you mean but its showing error
               float y = (BoomFixedY - PistonArray[i].y)/PistonFractionAlongBoomFixedToFinish + BoomFixedY;
		  	BoomArrayList.Add( new Vector3(x, y, 0f) );
		   }
        Vector3[] BoomArray = BoomArrayList.ToArray(); 

        if (BoomFinish == BoomStart)
         {
             List<Vector3> BoomArray2List = new List<Vector3>();
		     for ( int i = 0; i < ArrayLength; i++)
		     {
                float x = BoomArray[i].x + (BoomFixedX - BoomArray[i].x)*((1+BoomOverShootFraction)/BoomOverShootFraction);
                float y = BoomArray[i].y + (BoomFixedY - BoomArray[i].y)*((1+BoomOverShootFraction)/BoomOverShootFraction);
		     	BoomArray2List.Add( new Vector3(x, y, 0f) );
		     } 
          Vector3[] BoomArray2 = BoomArray2List.ToArray();
          return BoomArray2;
         }
         else
         {
          return BoomArray;
         }
        

    }

    // public static float DistanceBetweenPoints(Vector3 v1, Vector3 v2)
    //     {
    //         float xDiff = (float)Math.Pow((v2.X - v1.X),2.0);
    //         float yDiff = (float)Math.Pow((v2.Y - v1.Y),2.0);

    //         return (float)Math.Sqrt(xDiff + yDiff);
    //     }
    public Vector3[] BoomRotation(Vector3 BoomFixed, Vector3 BoomEnd, Vector3 PistonStart, float BoomOverShootFraction, float PistonFraction, float TimeStep)
        {
            float BoomFixedX = BoomFixed[0];
            float BoomFixedY = BoomFixed[1]; 
            float BoomEndX = BoomEnd[0];
            float BoomEndY = BoomEnd[1];
            float PistonStartX = PistonStart[0];
            float PistonStartY = PistonStart[1];

            Vector3 VectorBoomEndToBoomFixed = BoomFixed - BoomEnd;
            Vector3 BoomStart = BoomFixed + (BoomOverShootFraction*VectorBoomEndToBoomFixed);
            float BoomStartX = BoomStart[0];
            float BoomStartY = BoomStart[1];
            float PistonEndX = (BoomEndX-BoomStartX)*PistonFraction +BoomStartX;
            float PistonEndY = (BoomEndY-BoomStartY)*PistonFraction +BoomStartY; 
            
            Vector3 PistonEnd = new Vector3(PistonEndX, PistonEndY, 0f);
        
    //Data from inputs
            float BoomLength = DistanceBetweenPoints(BoomStart, BoomEnd);
            float BoomStartToPistonLink = BoomLength*PistonFraction;
            float BoomFixedToPistonStart = DistanceBetweenPoints(BoomFixed, PistonStart);
            

            Vector3 BoomFinish = new Vector3(BoomEndX, BoomEndY, 0);
            if (BoomStartToPistonLink < BoomLength*(BoomOverShootFraction/(BoomOverShootFraction+1)))
            {
            BoomFinish[0] = BoomStartX;
            BoomFinish[1] = BoomStartY;
            } 
            float BoomFinishX = BoomFinish[0];
            float BoomFinishY = BoomFinish[1];

    //Requied for solve
            float BoomFixedToFinished = DistanceBetweenPoints(BoomFixed, BoomFinish);
            float BoomFixedToPistonLink = DistanceBetweenPoints(BoomFixed, PistonEnd);
            float PistonFractionAlongBoomFixedToFinish = BoomFixedToPistonLink/BoomFixedToFinished;    

    //Piston Data
            float PistonRatio = 1.8f;
            float PistonRate = 0.2f;
            bool Max = false; //Assume piston placed in minimum position unless defined as maximum
            float PistonMin = DistanceBetweenPoints(PistonStart, PistonEnd);
            float PistonMax = 0f;

            if (Max == false)
            {
            if ((PistonMin*PistonRatio)<(BoomFixedToPistonStart+BoomFixedToPistonLink))
            {
            PistonMax = PistonMin*PistonRatio;
            }   
            else
            {
            PistonMax = BoomFixedToPistonStart+BoomFixedToPistonLink;
            }              
            }
            else if (Max == true)
            {
            PistonMax = DistanceBetweenPoints(PistonStart, PistonEnd);
            PistonMin = PistonMax/PistonRatio;
            }
    //Time Data
            float PistonStep = PistonRate*TimeStep;
            int ArrayLength = (int)Math.Round((PistonMax-PistonMin)/PistonStep);
            
    //Piston Length Array 
            float[] PistonLength = new float [ArrayLength];  
            for (int i = 0; i<ArrayLength; i++)
            {
            PistonLength[i] = PistonMin + i*PistonStep;
            }
            // Array.ForEach(PistonLength, Console.WriteLine);

    //Time Array 
            float[] Time = new float [ArrayLength];   
            for (int i = 0; i<ArrayLength; i++)
            {
                Time[i] = i*TimeStep;
            }
    //Solve 
    //z array
            float[] z = new float[ArrayLength];
            for (int i = 0; i<ArrayLength; i++)
            {
                z[i] = (((float)Math.Pow(BoomFixedToPistonLink,2.0) + (float)Math.Pow(BoomFixedToPistonStart,2.0) - (float)Math.Pow(PistonLength[i],2.0))/(2*BoomFixedToPistonLink*BoomFixedToPistonStart)); 
            }         
    //Theta - angle between Boom and vector BoomFixed to PistonStart - Alpha is the polar angle of Piston Start from Boom Fixed
            float Alpha = (float)Math.Atan2( (PistonStartY-BoomFixedY), (PistonStartX-BoomFixedX));
            float AlphaThetaOne = (float)Math.Atan2( (BoomFinishY-BoomFixedY), (BoomFinishX -BoomFixedX));

            int o1 = (int)Math.Round((AlphaThetaOne - Alpha - (float)Math.Acos(z[0]))*180/Math.PI);
            int o2 = (int)Math.Round((AlphaThetaOne - Alpha + (float)Math.Acos(z[0]))*180/Math.PI);
            int o3 = (int)Math.Round((AlphaThetaOne + Alpha - (float)Math.Acos(z[0]))*180/Math.PI);
            int o4 = (int)Math.Round((AlphaThetaOne + Alpha + (float)Math.Acos(z[0]))*180/Math.PI);
            // Console.WriteLine(o1);
            // Console.WriteLine(o2);
            // Console.WriteLine(o3);
            // Console.WriteLine(o4);

        float[] Theta = new float[ArrayLength];
            if (o1 == 0 || o1 == 360) 
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] = (float)Math.Acos(z[i]);
            }
         }
            if (o2 == 0 || o2 == 360) 
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  -(float)Math.Acos(z[i]);
            }
         }
            if (o3 == 0 || o3 == 360) 
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  (float)Math.Acos(z[i]) ;
            }
            Alpha = -Alpha;
         }
            if (o4 == 0 || o4 == 360) 
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  - (float)Math.Acos(z[i]);
            }
            Alpha = -Alpha;
         }

    //Using correct angles
    //Calc Piston Vectors
            Vector3[] PistonArray = new Vector3[ArrayLength];
		    for ( int i = 0; i < ArrayLength; i++)
		    {
                float x = BoomFixedToPistonLink*(float)Math.Cos(Theta[i]+Alpha) + BoomFixedX;
                float y = BoomFixedToPistonLink*(float)Math.Sin(Theta[i]+Alpha) + BoomFixedY;
		    	PistonArray[i] = new Vector3(x, y, 0f) ;
		    }

            Vector3[] BoomArray = new Vector3[ArrayLength];
		    for ( int i = 0; i < ArrayLength; i++)
		    {
                float x = (PistonArray[i][0] - BoomFixedX)/PistonFractionAlongBoomFixedToFinish + BoomFixedX;
                float y = (PistonArray[i][1] - BoomFixedY)/PistonFractionAlongBoomFixedToFinish + BoomFixedY;
		    	BoomArray[i] = new Vector3(x, y, 0f) ;
		    }
            if (BoomFinish == BoomStart)
            {
        
		    for ( int i = 0; i < ArrayLength; i++)
		    {
                float x = BoomArray[i][0] + (BoomFixedX - BoomArray[i][0])*((1+BoomOverShootFraction)/BoomOverShootFraction);
                float y = BoomArray[i][1]+ (BoomFixedY - BoomArray[i][1])*((1+BoomOverShootFraction)/BoomOverShootFraction);
		    	BoomArray[i] = new Vector3(x, y, 0f);
		    } 
            }
            
            return BoomArray;
           
        }




    public Vector3[] ConvertToVector3Array (float[] floats)
	{
		List<Vector3> vector3List = new List<Vector3>();
		for ( int i = 0; i < floats.Length; i += 3)
		{
			vector3List.Add( new Vector3(floats[i], floats[i+1], floats[i+2]) );
		}
		return vector3List.ToArray();
	}



    public async void Update()
    {
        float BoomOverShootFraction = 0f;
        float PistonFraction = 0.5f;
        float TimeStep = 50f;

        Vector3 joint1 = receivedPos5;
        Vector3 joint2 = receivedPos6;
        Vector3 piston1 = receivedPos7;
        Vector3 PistonOneEndPos = PistonEnd(joint1, joint2, PistonFraction);
	    Vector3[] BoomArray = BoomRotation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);
        //print(BoomArray[0]);
        //print(BoomArray.Length);
        BoomOneCurve.positionCount = BoomArray.Length;

        
         if(!joint1.Equals(outofframe)&&!joint2.Equals(outofframe))
        {
            Vector3[] LinePosition1 = {joint1,joint2};
            Line.SetPositions(LinePosition1);
        }
        else
        {
            Vector3[] LinePosition1 = {outofframe, outofframe};
            Line.SetPositions(LinePosition1);
        }
        

        if(!piston1.Equals(outofframe)&&!PistonOneEndPos.Equals(outofframe)&&!joint1.Equals(outofframe)&&!joint2.Equals(outofframe))
        {

            Vector3[] LinePosition2 = {piston1,PistonOneEndPos};
            PistonLine.SetPositions(LinePosition2);
        }
        else
        {
            Vector3[] LinePosition2 = {outofframe, outofframe};
            PistonLine.SetPositions(LinePosition2);
        }

    
        if(MenuData[0]==0){
        //edit
        BoomOneCurve.GetComponent<Renderer>().enabled=false;
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        // squareArray[0].transform.position = receivedPos5;
        // squareArray[1].transform.position = receivedPos6;
        // squareArray[2].transform.position = receivedPos7;
        if(joint1!=outofframe){
        squareArray[0].GetComponent<Renderer>().enabled = true;
        squareArray[0].transform.position = joint1;
        }else{
        squareArray[0].GetComponent<Renderer>().enabled = false;
        }
        if(joint2!=outofframe){
        squareArray[1].GetComponent<Renderer>().enabled = true;
        squareArray[1].transform.position = joint2;
        }else{
        squareArray[1].GetComponent<Renderer>().enabled = false;
        }
        if(piston1!=outofframe){
        squareArray[2].GetComponent<Renderer>().enabled = true;
        squareArray[2].transform.position = piston1;
        }else{
        squareArray[2].GetComponent<Renderer>().enabled = false;
        }
        if(!piston1.Equals(outofframe)&&!PistonOneEndPos.Equals(outofframe)&&!joint1.Equals(outofframe)&&!joint2.Equals(outofframe))
        {
            PistonLine.GetComponent<Renderer>().enabled = true;
            Vector3[] LinePosition2 = {piston1,PistonOneEndPos};
            PistonLine.SetPositions(LinePosition2);
        }
        else
        {
            PistonLine.GetComponent<Renderer>().enabled = false;
            //Vector3[] LinePosition2 = {outofframe, outofframe};
            //PistonLine.SetPositions(LinePosition2);
        }

        
        }
        

        
        else if(MenuData[0]==1){
        //animate
        BoomOneCurve.GetComponent<Renderer>().enabled=true;
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        //StartCoroutine(FollowArrayPath());
        for(int i=0; i<BoomArray.Length; i++){
            BoomOneCurve.SetPosition(i,BoomArray[i]);
        }
        }
        else if (MenuData[0]==2){
        //data
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.blue;   
        } 
        else{

        }
        //Link Menu
        if(LinkMenuData[0]==1){
        LinkArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
        LinkArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[3].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[4].GetComponent<SpriteRenderer>().material.color = Color.white;
        }
        else if (LinkMenuData[0]==2){
        LinkArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
        LinkArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[3].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[4].GetComponent<SpriteRenderer>().material.color = Color.white;
        }
        else if (LinkMenuData[0]==3){
        LinkArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[2].GetComponent<SpriteRenderer>().material.color = Color.blue;
        LinkArray[3].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[4].GetComponent<SpriteRenderer>().material.color = Color.white;
        }
        else if (LinkMenuData[0]==4){
        LinkArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[3].GetComponent<SpriteRenderer>().material.color = Color.blue;
        LinkArray[4].GetComponent<SpriteRenderer>().material.color = Color.white;
        }
        else if (LinkMenuData[0]==5){
        LinkArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[3].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[4].GetComponent<SpriteRenderer>().material.color = Color.blue;
        }
        else{
        LinkArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[3].GetComponent<SpriteRenderer>().material.color = Color.white;
        LinkArray[4].GetComponent<SpriteRenderer>().material.color = Color.white;

        }
        

       
    }
    public IEnumerator FollowArrayPath()
    {   
        float BoomOverShootFraction = 0f;
        float PistonFraction = 0.5f;
        float TimeStep = 50f;
        
        
        Vector3 joint1 = receivedPos5;
        Vector3 joint2 = receivedPos6;
        Vector3 piston1 = receivedPos7;
        Vector3 PistonOneEndPos = PistonEnd(joint1, joint2, PistonFraction);
	    Vector3[] BoomArray = BoomRotation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);
        Debug.Log("print"+BoomArray[0]);

        for(int i=0; i<=BoomArray.Length; i++)
        {
            Vector3 pos = BoomArray[i];
            yield return StartCoroutine(DrawLine(pos, 8));
        }
    }
    public IEnumerator DrawLine(Vector3 posonlink, float speed)
    {
        while(squareArray[1].transform.position != posonlink){
            squareArray[1].transform.position = Vector3.MoveTowards (squareArray[1].transform.position, posonlink, speed* Time.deltaTime);
            // Vector3[] LinePosition1 = {squareArray[0].transform.position, posonlink};
            // Line.SetPositions(LinePosition1);
            yield return null;
        }
       
    }
    
    
}