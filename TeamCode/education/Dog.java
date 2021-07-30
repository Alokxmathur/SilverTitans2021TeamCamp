package org.firstinspires.ftc.teamcode.education;

public abstract class Dog {
    private static String species="Dog";

    protected String name;
    private int ageInYears;


    public Dog(String name, int ageInYears) {
        this.name = name;
        this.ageInYears = ageInYears;
    }

    public abstract String getName();

    public int getAgeInYears() {
        return this.ageInYears;
    }

    public abstract void bark();

    public String getType() {
        return this.getClass().getSimpleName();
    }

}
