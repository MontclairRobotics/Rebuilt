package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

public class FuelField {
    public Translation2d[] fuelPositions;
    public Angle[] fuelAngles;

    public FuelField(Translation2d[] initialFuelPositions, Angle[] initialAngles) {
        this.fuelPositions = initialFuelPositions;
        this.fuelAngles = initialAngles;
    }

    public void update(String name) {
        this.fuelPositions = RobotContainer.vision.fieldFuelLocations(name);
        this.fuelAngles = RobotContainer.vision.angles(name);
    }

    public void filterFuel(double minDistance, double maxDistance) { //add way to filter out fuel on opposing side
        int newSize = this.fuelPositions.length;
        for(int i = 0; i < this.fuelPositions.length; i++) {
            if(Math.sqrt(Math.pow(this.fuelPositions[i].getX(),2) + Math.pow(this.fuelPositions[i].getY(),2)) > maxDistance ||
            Math.sqrt(Math.pow(this.fuelPositions[i].getX(),2) + Math.pow(this.fuelPositions[i].getY(),2)) < minDistance) {
                newSize--;
            }
        }

        Translation2d[] newFuelPositions = new Translation2d[newSize];
        Angle[] newAngles = new Angle[newSize];
        int j = 0;
        for(int i = 0; i < this.fuelPositions.length; i++) {
            if(Math.sqrt(Math.pow(this.fuelPositions[i].getX(),2) + Math.pow(this.fuelPositions[i].getY(),2)) > maxDistance ||
            Math.sqrt(Math.pow(this.fuelPositions[i].getX(),2) + Math.pow(this.fuelPositions[i].getY(),2)) < minDistance) {
                newFuelPositions[j] = this.fuelPositions[i];
                newAngles[j] = this.fuelAngles[i];
                j++;
            }
        }

        this.fuelPositions = newFuelPositions;
        this.fuelAngles = newAngles;
    }

    public Translation2d chooseLocation(String cameraName, double samplingRate, int minDistance, int maxDistance) { //The sampling rate is how much the filtered distance changes each loop
        //Not at all mathematically optimal, just an approx. of the best straight line to go in
        if(RobotContainer.vision.isFuelVisible(cameraName)) {
            return new Translation2d(0,0);
        }
        double slope; //A simple way to go through the densest area is a line of best fit
        double slopeNumerator = 0;
        double slopeDenom = 0;

        for(int i = 0; i < this.fuelPositions.length; i++) {
            slopeNumerator += this.fuelPositions[i].getX() * this.fuelPositions[i].getY();
            slopeDenom += Math.pow(this.fuelPositions[i].getX(),2);
        }

        slope = slopeNumerator / slopeDenom;
        double distance; //We want to find a good distance to go. If we go too far we waste time and if we don't go far enough we don't get enough fuel
        
        int[] numsOfFuel = new int[(int) ((maxDistance - minDistance) / samplingRate) - 1];
        for(int i = minDistance; i < maxDistance - samplingRate; i += samplingRate) {
            numsOfFuel[i] = RobotContainer.vision.getFilteredPositions(this.fuelPositions, minDistance, i + samplingRate).length - RobotContainer.vision.getFilteredPositions(this.fuelPositions, minDistance, i).length;
        }

        int maxInd = 0;

        for(int i = 1; i < numsOfFuel.length; i++) {
            if(numsOfFuel[i] > numsOfFuel[maxInd]) {
                maxInd = i;
            }
        }

        return new Translation2d(Math.sqrt(Math.pow(maxInd * samplingRate + minDistance,2) / (Math.pow(slope,2) + 1)), slope * Math.sqrt(Math.pow(maxInd * samplingRate + minDistance,2) / (Math.pow(slope,2) + 1)));
    }
    
}
