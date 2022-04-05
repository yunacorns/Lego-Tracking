using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System;

Vector3 BoomStart = new Vector3(200,-157,0);
Vector3 BoomEnd = new Vector3(400,-200,0);

public float DistanceBetweenPoints(Vector3 v1, Vector3 v2)
        {
            float xDiff = (float)Math.Pow((v2[0] - v1[0]),2.0);
            float yDiff = (float)Math.Pow((v2[1] - v1[1]),2.0);

            return (float)Math.Sqrt(xDiff + yDiff);
        }
public float BoomLength(){
float BoomLength = DistanceBetweenPoints(BoomStart, BoomEnd);
Console.WriteLine(DistanceBetweenPoints)
}
