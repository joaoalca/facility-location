package org.example;
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;

import java.util.ArrayList;
import java.util.List;


// docker run -it --rm -v src/main/java/org/example or-tools-image java -cp Solver main

public class Solver {
    static {
        System.loadLibrary("jniortools");
    }

    static class Point {
        public double x;
        public double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    static class Facility {
        public int index;
        public double setup_cost;
        public int capacity;
        public Point location;

        public Facility(int index, double setup_cost, int capacity, Point location) {
            this.index = index;
            this.setup_cost = setup_cost;
            this.capacity = capacity;
            this.location = location;
        }
    }

    static class Customer {
        public int index;
        public int demand;
        public Point location;

        public Customer(int index, int demand, Point location) {
            this.index = index;
            this.demand = demand;
            this.location = location;
        }
    }

    // Method to calculate distance between two points
    public static double length(Point point1, Point point2) {
        return Math.sqrt(Math.pow((point1.x - point2.x), 2) + Math.pow((point1.y - point2.y), 2));
    }

    public static void main(String[] args) {
        System.out.println("Hello world!");
    }

    public static String solve_it(String input_data) {
        // The existing code of your method up to the point where facilities and customers are initialized...
        // Splitting input data into lines and parts
        String[] lines = input_data.split("\n");
        String[] parts = lines[0].split("\\s+");

        // get count of facilities and customers
        int facility_count = Integer.parseInt(parts[0]);
        int customer_count = Integer.parseInt(parts[1]);

        // Initializing facilities list
        List<Facility> facilities = new ArrayList<>();
        for (int i = 1; i <= facility_count; i++) {
            parts = lines[i].split("\\s+");
            facilities.add(new Facility(i - 1, Double.parseDouble(parts[0]), Integer.parseInt(parts[1]),
                    new Point(Double.parseDouble(parts[2]), Double.parseDouble(parts[3]))));
        }

        // Initializing customers list
        List<Customer> customers = new ArrayList<>();
        for (int i = facility_count + 1; i <= facility_count + customer_count; i++) {
            parts = lines[i].split("\\s+");
            customers.add(new Customer(i - 1 - facility_count, Integer.parseInt(parts[0]),
                    new Point(Double.parseDouble(parts[1]), Double.parseDouble(parts[2]))));
        }

        // Create the linear solver with the SCIP backend.
        MPSolver solver = MPSolver.createSolver("SCIP");

        // Decision variable x[f] for each facility f, indicating whether it is open.
        MPVariable[] x = new MPVariable[facility_count];
        for (int i = 0; i < facility_count; i++) {
            x[i] = solver.makeIntVar(0, 1, "x[" + i + "]");
        }

        // Decision variable y[c, f] for each customer c and facility f, indicating whether c is assigned to f.
        MPVariable[][] y = new MPVariable[customer_count][facility_count];
        for (int i = 0; i < customer_count; i++) {
            for (int j = 0; j < facility_count; j++) {
                y[i][j] = solver.makeIntVar(0, 1, "y[" + i + "," + j + "]");
            }
        }

        // Objective function: minimize setup cost and delivery cost.
        MPObjective objective = solver.objective();
        for (int i = 0; i < facility_count; i++) {
            objective.setCoefficient(x[i], facilities.get(i).setup_cost);
            for (int j = 0; j < customer_count; j++) {
                objective.setCoefficient(y[j][i], length(customers.get(j).location, facilities.get(i).location));
            }
        }
        objective.setMinimization();

        // Constraints
        // (1) Each customer must be served by exactly one facility.
        for (int i = 0; i < customer_count; i++) {
            MPConstraint constraint1 = solver.makeConstraint(1, 1);
            for (int j = 0; j < facility_count; j++) {
                constraint1.setCoefficient(y[i][j], 1);
            }
        }

        // (2) A facility can only serve a customer if it is open.
        for (int i = 0; i < facility_count; i++) {
            for (int j = 0; j < customer_count; j++) {
                MPConstraint constraint2 = solver.makeConstraint(0, 0);
                constraint2.setCoefficient(y[j][i], 1);
                constraint2.setCoefficient(x[i], -1);
            }
        }

        // (3) A facility cannot serve more than its capacity.
        for (int i = 0; i < facility_count; i++) {
            MPConstraint constraint3 = solver.makeConstraint(0, facilities.get(i).capacity);
            for (int j = 0; j < customer_count; j++) {
                constraint3.setCoefficient(y[j][i], customers.get(j).demand);
            }
        }

        // Solve the problem and print the solution.
        MPSolver.ResultStatus status = solver.solve();
        if (status != MPSolver.ResultStatus.OPTIMAL) {
            return "The problem does not have an optimal solution.";
        }

        // Calculate the total cost.
        double totalCost = 0;
        for (int i = 0; i < facility_count; i++) {
            totalCost += x[i].solutionValue() * facilities.get(i).setup_cost;
            for (int j = 0; j < customer_count; j++) {
                totalCost += y[j][i].solutionValue() * length(customers.get(j).location, facilities.get(i).location);
            }
        }

        // Identify which customers are assigned to which facilities.
        int[] assignments = new int[customer_count];
        for (int i = 0; i < customer_count; i++) {
            for (int j = 0; j < facility_count; j++) {
                if (y[i][j].solutionValue() > 0.5) {
                    assignments[i] = j;
                    break;
                }
            }
        }

        // Build the output string.
        StringBuilder output = new StringBuilder();
        output.append(totalCost).append(" ").append(1).append("\n");
        for (int i = 0; i < customer_count; i++) {
            output.append(assignments[i]).append(" ");
        }
        output.append("\n");

        return output.toString();
    }

}