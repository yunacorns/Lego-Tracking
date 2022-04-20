if (InMenuRegion(0, 100, -70, -120) == "in range") // edit
        {
            //Menu colour changes and enabling of things appearing
            BoomOneCurve.GetComponent<Renderer>().enabled=false;
            editSubMenuArray.GetComponent<Renderer>().enabled=true; //.transform.position of sub menu or enabling appearance
            menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
            menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
            menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
            //render components only if they exist
            if(joint1!=outofframe) //joint 1 existance
            {
                squareArray[0].GetComponent<Renderer>().enabled = true;
                squareArray[0].transform.position = joint1;
            }
            else
            {
                squareArray[0].GetComponent<Renderer>().enabled = false;
            }
            if(joint2!=outofframe) //joint 2 existance
            {
                squareArray[1].GetComponent<Renderer>().enabled = true;
                squareArray[1].transform.position = joint2;
                BoomOneEnd.transform.position = joint2;
            }
            else
            {
                squareArray[1].GetComponent<Renderer>().enabled = false;
            }
            if(piston1!=outofframe) //piston 1 existance
            {
                squareArray[2].GetComponent<Renderer>().enabled = true;
                squareArray[2].transform.position = piston1;
            }
            else
            {
                squareArray[2].GetComponent<Renderer>().enabled = false;
            }
            if(!joint1.Equals(outofframe)&&!joint2.Equals(outofframe)) //link 1 existance
            {
                Line.GetComponent<Renderer>().enabled = true;
                Vector3[] LinePosition1 = {BoomStart,joint2};
                Line.SetPositions(LinePosition1);
            }
            else
            {
                Line.GetComponent<Renderer>().enabled = false;
                Vector3[] LinePosition1 = {outofframe, outofframe};
                Line.SetPositions(LinePosition1);
            }

            if(!piston1.Equals(outofframe)&&!joint1.Equals(outofframe)&&!joint2.Equals(outofframe)) //piston 1 link existance
            {
                PistonLine.GetComponent<Renderer>().enabled = true;
                Vector3[] LinePosition2 = {piston1,PistonOneEndPos};
                PistonLine.SetPositions(LinePosition2);
            }
            else
            {
                PistonLine.GetComponent<Renderer>().enabled = false;
            }


            InSubMenuPiston = InMenuRegion(628,712,-500,-590,receivedPos26);

            if (InSubMenuPiston == "in range") //pistons selected
            {
                //need a .transform.position of piston selector menu
                subMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                subMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                if (InMenuRegion(850, 950, -70, -120) == "in range") //1st piston
                {
                    selectorHighlighter.tranform.postion = new vector3(850,-95,0); //highlights that the piston has been selected
                    // need to add prompts appear along bottom describing function of slider
                    if (Slider_Position[0] < 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(275,-585,0); //highlights that the slider is in this half
                        float PistonFraction1 = sliderValue(receivedPos9,receivedPos8);// slider used for piston1 positioning
                    }
                    else if (Slider_Position[0] > 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(675,-585,0);
                        float PistonExtension1 = sliderValue(receivedPos9,receivedPos8);// slider used for piston1 extension edit
                    }
                }
                else if (InMenuRegion(850, 950, -120, -170) == "in range") //2nd piston
                {
                    selectorHighlighter.tranform.postion = new vector3(850,-145,0);
                    if (Slider_Position[0] < 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(275,-585,0);
                        float PistonFraction2 = sliderValue(receivedPos9,receivedPos8);// slider used for piston2 positioning
                    }
                    else if (Slider_Position[0] > 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(675,-585,0);
                        float PistonExtension2 = sliderValue(receivedPos9,receivedPos8);// slider used for piston2 extension edit
                    }
                }
                else if (InMenuRegion(850, 950, -170, -220) == "in range") //3rd piston
                {
                    selectorHighlighter.tranform.postion = new vector3(850,-195,0);
                    if (Slider_Position[0] < 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(275,-585,0);
                        float PistonFraction3 = sliderValue(receivedPos9,receivedPos8); // slider used for piston3 positioning
                    }
                    else if (Slider_Position[0] > 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(675,-585,0);
                        float PistonExtension3 = sliderValue(receivedPos9,receivedPos8);// slider used for piston3 extension edit
                    }
                }
                else if (InMenuRegion(850, 950, -220, -270) == "in range") //4th piston
                {
                    selectorHighlighter.tranform.postion = new vector3(850,-245,0);
                    if (Slider_Position[0] < 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(275,-585,0);
                        float PistonFraction4 = sliderValue(receivedPos9,receivedPos8);// slider used for piston4 positioning
                    }
                    else if (Slider_Position[0] > 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(675,-585,0);
                        float PistonExtension4 = sliderValue(receivedPos9,receivedPos8);// slider used for piston extension edit
                    }
                }
            }
            else if (InMenuRegion(0, 100, -400,-450) == "in range") //links
            {
                //.transform.position of piston selector menu
                subMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                subMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                if (InMenuRegion(850, 950, -70, -120) == "in range") //1st link
                {
                    selectorHighlighter.tranform.postion = new vector3(850,-95,0);
                    if (Slider_Position[0] < 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(275,-585,0);
                        BoomFractionOvershoot1 = sliderValue(receivedPos9,receivedPos8);// slider used for link extension
                    }
                }
                else if (InMenuRegion(850, 950, -120, -170) == "in range") //2nd link
                {
                    selectorHighlighter.tranform.postion = new vector3(850,-145,0);
                    if (Slider_Position[0] < 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(275,-585,0);
                        BoomFractionOvershoot2 = sliderValue(receivedPos9,receivedPos8);// slider used for link extension
                    }
                    else if (Slider_Position[0] > 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(675,-585,0);
                        JointFraction2 = sliderValue(receivedPos9,receivedPos8);// slider used for next link start
                    }
                }
                else if (InMenuRegion(850, 950, -170, -220) == "in range") //3rd link
                {
                    selectorHighlighter.tranform.postion = new vector3(850,-195,0);
                    if (Slider_Position[0] < 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(275,-585,0);
                        BoomFractionOvershoot3 = sliderValue(receivedPos9,receivedPos8);// slider used for link extension
                    }
                    else if (Slider_Position[0] > 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(675,-585,0);
                        JointFraction3 = sliderValue(receivedPos9,receivedPos8);// slider used for next link start
                    }
                }
                else if (InMenuRegion(850, 950, -220, -270) == "in range") //4th link
                {
                    selectorHighlighter.tranform.postion = new vector3(850,-245,0);
                    if (Slider_Position[0] < 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(275,-585,0);
                        BoomFractionOvershoot4 = sliderValue(receivedPos9,receivedPos8);// slider used for link extension
                    }
                    else if (Slider_Position[0] > 495)
                    {
                        sliderPositionHighlighter.tranform.postion = new vector3(675,-585,0);
                        JointFraction4 = sliderValue(receivedPos9,receivedPos8);// slider used for next link start
                    }
                }
            }
        }
