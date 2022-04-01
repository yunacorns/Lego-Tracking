using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System;

[RequireComponent(typeof(LineRenderer))]


public class trial : MonoBehaviour
{
    public GameObject[] squareArray;
    public GameObject PistonOneEnd;
    public LineRenderer Line;
    public LineRenderer PistonLine;
    public LineRenderer BoomOneCurve;

    
    Thread mThread;
    public string connectionIP = "127.0.0.1";
    public int connectionPort = 25001;
    IPAddress localAdd;
    TcpListener listener;
    TcpClient client;
    Vector3 receivedPos5 = new Vector3(-100,0,0);
    Vector3 receivedPos6 = new Vector3(-100,0,0);
    Vector3 receivedPos7 = new Vector3(-100,0,0);
    Vector3 size = Vector3.zero;
    Vector3 zeros = Vector3.zero;
    Vector3 outofframe = new Vector3(-100,0,0);


    public bool running; //nothing

    public void Start() //private
    {
        ThreadStart ts = new ThreadStart(GetInfo);
        mThread = new Thread(ts);
        mThread.Start();

        // float PistonFractionOne = 0.5f;
        // Vector3[] BoomOnePos = Calculations(receivedPos3, receivedPos4, receivedPos5, PistonFractionOne);
        // BoomOneCurve.positionCount = BoomOnePos.Length;
        BoomOneCurve.positionCount = 3;
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
            
            receivedPos5 = StringToVector3(dataReceived,"5");
            receivedPos6 = StringToVector3(dataReceived,"6");
            receivedPos7 = StringToVector3(dataReceived,"7");

            print("received position one data, and moved the Cube!");
            
            

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
    // public static string[] GetCorrectData(string sVector){
    //     if (sVector.StartsWith("(") && sVector.EndsWith(")"))
    //     {
    //     sVector = sVector.Substring(1, sVector.Length - 2);
    //     }
    //     string[] sArray = sVector.Split(',');
    //     if(sArray[0]=="5"){
    //         return sArray;
    //     }
    //     else if (sArray[0]=="6"){
    //         return sArray;
    //     }else{
    //         return sArray;
    //     }
       
    // }
    


    
    // public static Vector3 StringToVector3(string sVector, string whichSquare)
    // {
    //     // Remove the parentheses
    //     if (sVector.StartsWith("(") && sVector.EndsWith(")"))
    //     {
    //     sVector = sVector.Substring(1, sVector.Length - 2);
    //     }
    //     string[] sArray = sVector.Split(',');
    //     if(sArray[1]!=whichSquare)
    //     {
    //         Vector3 result = new Vector3(-100,-100,0);
    //         return result;
    //     }else{
    //         Vector3 result = new Vector3(
    //         float.Parse(sArray[1]),
    //         float.Parse(sArray[2]),
    //         float.Parse(sArray[3]));
    //         return result;
    //     }
        

    // }
    


// // // Circle code here
//     public int vertexCount = 40;
//     public float lineWidth = 0.2f;
//     private float radius;
//     private LineRenderer lineRenderer;

// #if UNITY_EDITOR
//     public void OnDrawGizmos()
//     {

//         radius = (float)Math.Sqrt((Math.Pow(receivedPos3[0]-receivedPos4[0],2)+Math.Pow(receivedPos3[1]-receivedPos4[1],2)));

//         float deltaTheta = (2f * Mathf.PI) / vertexCount;
//         float theta = 0f;

//         Vector3 oldPos = Vector3.zero;
//         for (int i = 0; i<vertexCount +1; i++)
//         {
//             Vector3 pos = new Vector3(radius * Mathf.Cos(theta), radius * Mathf.Sin(theta), 0f);
//             Gizmos.DrawLine(oldPos, transform.position + pos);
//             transform.position = receivedPos4;
//             oldPos = transform.position + pos;

//             theta += deltaTheta;
//         }
//     }
// #endif

// // Animation here

//     public void Main(string[] args)
//     {
//         CreateArray();
//     }

//     public void CreateArray()
//     {

//         Vector3[] pointsoncircle = new [] { 
//             new Vector3(receivedPos3[0],(receivedPos3[1]+radius) , 0f), 
//             new Vector3((receivedPos3[0]+radius),(receivedPos3[1]) , 0f), 
//             new Vector3(receivedPos3[0],(receivedPos3[1]-radius) , 0f), 
//             new Vector3((receivedPos3[0]-radius),(receivedPos3[1]) , 0f)};      

//     }

// // George's Calculation here

    public float DistanceBetweenPoints(Vector3 v1, Vector3 v2)
        {
            float xDiff = (float)Math.Pow((v2[0] - v1[0]),2.0);
            float yDiff = (float)Math.Pow((v2[1] - v1[1]),2.0);

            return (float)Math.Sqrt(xDiff + yDiff);
        }

    
    public Vector3[] ConvertToVector3Array (float[] floatX, float[] floatY)
    {
        Vector3[] PositionArray = new Vector3[floatX.Length];
        for (int i=0; i<floatX.Length;i++)
        {
            PositionArray[i]= new Vector3(floatX[i],floatY[i],0f);
        }
        return PositionArray;
        
    }
	    // {
		//     float[] ZerosArray = new float [floatX.Length];
        //     for (int k=0; k<floatX.Length; k++)
        //     {
        //         ZerosArray[k] = 0f;
        //     }
        //     List<Vector3> vector3List = new List<Vector3>();
		//     for ( int i = 0; i < floatX.Length; i++)
		//     {
		//     	vector3List.Add( new Vector3(floatX[i], floatY[i], ZerosArray[i]) );
		//     }
		//     return vector3List.ToArray();
	    // }
    public float Calculations(Vector3 BoomOneStart, Vector3 BoomOneEnd, Vector3 PistonOneStart, float PistonOneFraction)
    {
        // // defining variable/coordinate
        // Vector3 BoomOneStart = receivedPos3;
        // Vector3 BoomOneEnd = receivedPos4;
        // Vector3 PistonOneStart = receivedPos5;
        

        //start c
        float BoomOneStartX = BoomOneStart[0];
        float BoomOneStartY = BoomOneStart[1]; 
        float BoomOneEndX = BoomOneEnd[0];
        float BoomOneEndY = BoomOneEnd[1];
        float PistonOneStartX = PistonOneStart[0];
        float PistonOneStartY = PistonOneStart[1];
        //end c


        // float PistonOneFraction = 0.8f;

        float PistonOneEndX = PistonOneFraction*(BoomOneEndX - BoomOneStartX) + BoomOneStartX;
        float PistonOneEndY = PistonOneFraction*(BoomOneEndY - BoomOneStartY) + BoomOneStartY;

        Vector3 PistonOneEnd = new Vector3(PistonOneEndX, PistonOneEndY, 0f);

        // data from inputs
        float BoomOneLength = DistanceBetweenPoints(BoomOneStart, BoomOneEnd);
        float BoomLengthToPiston = BoomOneLength*PistonOneFraction;
        float BoomOneStartToPistonOneStart = DistanceBetweenPoints(BoomOneStart, PistonOneStart);

        //Piston Constants - finding minimum and maximum length of piston
        const float PistonRatio = 1.8f;
        const float PistonRate = 0.5f;
        //bool Max = false; //Assume piston placed in minimum position unless defined as maximum
        float PistonOneMin = DistanceBetweenPoints(PistonOneStart, PistonOneEnd);
        float PistonOneMax = 0f;
            //if (Max = false)
            //{
                if ((PistonOneMin*PistonRatio)<(BoomOneStartToPistonOneStart+BoomLengthToPiston))
                {
                    PistonOneMax = PistonOneMin*PistonRatio;
                }   
                 else
                {
                    PistonOneMax = BoomOneStartToPistonOneStart+BoomLengthToPiston;
                }
                
            //}
            //else if (Max = true)
            //{
            //    PistonOneMax = DistanceBetweenPoints(PistonOneStart, PistonOneEnd);
            //    PistonOneMin = PistonOneMax/PistonRatio;
            //}

        //Time Step and Piston Step
        float TimeStep = 1f;
        float PistonStep = PistonRate*TimeStep;
        int ArrayLengthOne = (int)Math.Round((PistonOneMax-PistonOneMin)/PistonStep);
        float[] PistonOneLength = new float [ArrayLengthOne];
            
            for (int i = 0; i<PistonOneLength.Length; i++)
            {
                
                PistonOneLength[i] = PistonOneMin + i*PistonStep;
            }

            float[] Time = new float[ArrayLengthOne];
            for (int i=0; i<Time.Length; i++)
            {
                Time[i] = i*TimeStep;
            }

        //Solve
        //z array (simplifies calcucations)
        float[] z = new float [ArrayLengthOne];
            for (int i=0; i<z.Length; i++)
            {
                z[i] = (((float)Math.Pow(BoomLengthToPiston,2.0) + 
                (float)Math.Pow(BoomOneStartToPistonOneStart,2.0) - 
                (float)Math.Pow(PistonOneLength[i],2.0))/(2*BoomLengthToPiston*BoomOneStartToPistonOneStart));
            } 

        //Theta - angle between Boom 1 and vector BoomOneStart to PistonOneStart
        float[] Theta = new float [ArrayLengthOne];
            for (int i=0; i<Theta.Length; i++)
            {
                if (((PistonOneStartY-BoomOneStartY)/(PistonOneStartX-BoomOneStartX))> ((BoomOneEndY-BoomOneStartY)/(BoomOneEndX-BoomOneStartX)))
                {
                    if (PistonOneStartX < BoomOneStartX)
                    {
                        Theta[i] = (float)Math.Acos(z[i]) + (float)Math.PI; 
                    }
                    else
                    {
                        Theta[i] = -(float)Math.Acos(z[i]);
                    }
                }
                else
                {
                    if (PistonOneStartX < BoomOneStartX)
                    {
                        Theta[i] = -(float)Math.Acos(z[i]) + (float)Math.PI;
                    }
                    else
                    {
                        Theta[i] = (float)Math.Acos(z[i]);
                    }
                }

            }

        //Theta is in radians
        float Alpha = (float)Math.Atan((PistonOneStartY-BoomOneStartY)/(PistonOneStartY-BoomOneStartX));
        float[] PistonOneX = new float [ArrayLengthOne];
        float[] PistonOneY = new float [ArrayLengthOne];
        float[] BoomOneX = new float [ArrayLengthOne];
        float[] BoomOneY = new float [ArrayLengthOne];
        
           
            for (int j=0; j<PistonOneX.Length; j++)
            {
                PistonOneX[j] = BoomLengthToPiston*((float)Math.Cos(Alpha+Theta[j])) + BoomOneStartX;
                PistonOneY[j] = BoomLengthToPiston*((float)Math.Sin(Alpha+Theta[j])) + BoomOneStartY;
                BoomOneX[j] = (PistonOneX[j] - BoomOneStartX)/PistonOneFraction + BoomOneStartX;
                BoomOneY[j] = (PistonOneY[j] - BoomOneStartY)/PistonOneFraction + BoomOneStartY;
            }

        Vector3[] BoomOnePos = ConvertToVector3Array(BoomOneX, BoomOneY);
        //  return BoomOnePos;
        return PistonOneMax;

        // Vector3[] PistonOnePos = ConvertToVector3Array(PistonOneX, PistonOneY);

       
        // return PistonOnePos;

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
    // a function to make the line renderer to the end of the circle
    // public Vector3 SetCircleEnd(Vector3 CircleEnd)
    // {
    //         int radius = 40;
    //         Vector3 result = new Vector3(
    //         CircleEnd[0]-radius),
    //         float.Parse(sArray[index+2]),
    //         float.Parse(sArray[index+3]));
            
    //         return result;
    // }    

    public async void Update()
    {
        float PistonFractionOne = 0.5f;

        Vector3 joint1 = receivedPos5;
        Vector3 joint2 = receivedPos6;
        Vector3 piston1 = receivedPos7;
        Vector3 PistonOneEndPos = PistonEnd(joint1, joint2, PistonFractionOne);
        //Vector[] BoomOnePos = Calculations(receivedPos3, receivedPos4, receivedPos5, PistonFractionOne);
        float PistonMax =  Calculations(joint1, joint2, piston1, PistonFractionOne);
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

        if(!piston1.Equals(outofframe)&&!PistonOneEndPos.Equals(outofframe))
        {
            Vector3[] LinePosition2 = {piston1,PistonOneEndPos};
            PistonLine.SetPositions(LinePosition2);
        }
        else
        {
            Vector3[] LinePosition2 = {outofframe, outofframe};
            PistonLine.SetPositions(LinePosition2);
        }

        // for(int i=0; i<BoomOnePos.Length; i++)
        // {
        //     BoomOneCurve.SetPosition(i, BoomOnePos[i]);
        // }
        
        // squareone.transform.position = receivedPos1; //assigning receivedPos in SendAndReceiveData()
        // squaretwo.transform.position = receivedPos2;
        // squarethree.transform.position = receivedPos3;
        // squarefour.transform.position = receivedPos4;
        squareArray[0].transform.position = receivedPos5;
        squareArray[1].transform.position = receivedPos6;
        squareArray[2].transform.position = receivedPos7;
        // PistonOneEnd.transform.position = PistonOneEndPos;


        // squareone.transform.localScale = size; //assigning receivedPos in SendAndReceiveData()
        // squaretwo.transform.localScale = size;
        // squarethree.transform.localScale = size;
        // squarefour.transform.localScale = size;
        // squarefive.transform.localScale = size;

        // Debug.Log("print" + PistonMax);
        // Vector3 trial1 = BoomOnePos[0];
        // Vector3 trial2 = BoomOnePos[1];
        // Vector3 trial3 = BoomOnePos[2];
        // BoomOneCurve.SetPosition(0, trial1);
        // BoomOneCurve.SetPosition(1, trial2);
        // BoomOneCurve.SetPosition(2, trial3);
        

       
    }
}