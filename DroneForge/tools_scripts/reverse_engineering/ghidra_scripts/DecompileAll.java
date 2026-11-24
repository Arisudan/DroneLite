import java.io.PrintWriter;

import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.app.script.GhidraScript;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;

// Decompiles every function and writes C pseudocode to the path given as script argument 0.
public class DecompileAll extends GhidraScript {
	@Override
	public void run() throws Exception {
		String[] args = getScriptArgs();
		DecompInterface ifc = new DecompInterface();
		ifc.openProgram(currentProgram);
		int ok = 0, fail = 0;
		try (PrintWriter out = new PrintWriter(args[0])) {
			FunctionIterator it = currentProgram.getFunctionManager().getFunctions(true);
			while (it.hasNext() && !monitor.isCancelled()) {
				Function f = it.next();
				DecompileResults r = ifc.decompileFunction(f, 30, monitor);
				out.println("// ==== " + f.getEntryPoint() + " " + f.getName() + " size=" + f.getBody().getNumAddresses());
				if (r.decompileCompleted()) {
					out.println(r.getDecompiledFunction().getC());
					ok++;
				}
				else {
					out.println("// decompile failed: " + r.getErrorMessage());
					fail++;
				}
			}
		}
		println("DecompileAll done: ok=" + ok + " fail=" + fail);
	}
}
