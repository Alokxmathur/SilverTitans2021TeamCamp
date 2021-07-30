package org.firstinspires.ftc.teamcode.education;

public class PincherChihuahua extends Dog {
    public PincherChihuahua(String name, int ageInYears) {
        super(name, ageInYears);
    }

    @Override
    public void bark() {
        System.out.println("Woof woof woof woof");
    }

    public String getName() {
        return this.name;
    }

}
