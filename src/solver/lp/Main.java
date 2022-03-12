package solver.lp;

import ilog.concert.IloException;

import java.io.FileNotFoundException;

import java.nio.file.Path;
import java.nio.file.Paths;

public class Main
{  
  public static void main(String[] args) throws IloException
  {
	if(args.length == 0)
	{
		System.out.println("Usage: java Main <file>");
		return;
	}

	String input = args[0];
	Path path = Paths.get(input);
	String filename = path.getFileName().toString();
	System.out.println("Instance: " + input);

	Timer watch = new Timer();
	watch.start();
	LPInstance instance = new LPInstance(input);
	instance.solve();
	watch.stop();

	System.out.println("{\"Instance\": \"" + filename +
			"\", \"Time\": " + String.format("%.2f",watch.getTime()) +
			", \"Result\": \"" + instance.objectiveValue +
			"\", \"Solution\": \"OPT\"}");
  }
}