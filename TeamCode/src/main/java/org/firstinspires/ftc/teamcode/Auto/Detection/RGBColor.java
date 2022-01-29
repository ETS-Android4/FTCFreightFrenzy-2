package org.firstinspires.ftc.teamcode.Auto.Detection;


public class RGBColor {
    private int red = 0;
    private int green = 0;
    private int blue = 0;

    public RGBColor(){

    }
    public RGBColor(int r, int g, int b){
        red = r;
        green = g;
        blue=  b;
    }

    public void setRed(int r)
    {
        red = r;
    }
    public void setGreen(int g) {
        green = g;
    }
    public void setBlue(int b)
    {
        blue = b;
    }


    public int getRed(){
        return red;
    }
    public int getGreen(){
        return green;
    }
    public int getBlue(){
        return blue;
    }
    public int getBlack() { return -(green + blue + red); } //this makes a black value
    public int getYellow(){
        return green + blue;
    }
}