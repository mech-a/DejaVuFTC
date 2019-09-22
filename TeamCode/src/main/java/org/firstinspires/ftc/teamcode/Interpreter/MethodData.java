package org.firstinspires.ftc.teamcode.Interpreter;

import java.util.Arrays;

public class MethodData {
    public String className;
    public String functionName;
    public Object[] functionParams;

    public MethodData(String cName, String fName, Object... args) {
        functionName = fName;
        className = cName;
        functionParams = args;
    }

    public String toString() {
        return "name of class: " + className + " || name of function: " + functionName + " || params: " + Arrays.toString(functionParams);
    }

    public Class<?>[] getParamTypes() {
        Class<?>[] paramTypes = new Class[functionParams.length];
        for(int i = 0; i<functionParams.length; i++) {
            paramTypes[i] = functionParams[i].getClass();
        }
        return paramTypes;
    }

    //TODO find a good way to do this that isn't memory destroying and hopefully O(n)
    public void convertObjectsToPrimitives() {
        //double x = Double.parseDouble();
    }

    public Class<?> getClassObj() {
        //output: package name testing class ReflectionTesting.MethodData
        //System.out.println("package name testing " + this.getClass());

        //TODO find right package
        try{
            Class<?> c = Class.forName("ReflectionTesting" + "." +className);
            return c;
        }
        catch(Exception e) {
            e.printStackTrace();
        }
        return null;
    }
}
