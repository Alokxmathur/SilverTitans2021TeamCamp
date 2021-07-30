package org.firstinspires.ftc.teamcode.education;

public class Test {
    public static void main(String args[]) {
        System.out.println("This is a test");
        System.out.println("This is only a test");

        Dog dog1;
        Dog dog2;

        String nameOfTest = "Dog test";

        System.out.println("Name of test=" + nameOfTest);

        dog1 = new PincherChihuahua("Finn", 5);
        dog2 = new ShepherdHound("Chelsea", 7);

        System.out.println("Name of dog 1=" + dog1.getName());
        System.out.println("Age of dog 1=" + dog1.getAgeInYears());
        dog1.bark();

        System.out.println("Name of dog 2=" + dog2.getName());
        System.out.println("Age of dog 2=" + dog2.getAgeInYears());
        dog2.bark();
    }
}
