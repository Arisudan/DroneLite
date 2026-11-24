import java.io.PrintWriter;
import java.util.regex.Pattern;

import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.app.script.GhidraScript;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;

// Decompiles functions whose name (or namespace) matches a regex.
// args: <output path> <regex> [max functions]
public class DecompileMatching extends GhidraScript {
	@Override
	public void run() throws Exception {
		String[] args = getScriptArgs();
		Pattern p = Pattern.compile(args[1], Pattern.CASE_INSENSITIVE);
		int max = args.length > 2 ? Integer.parseInt(args[2]) : 400;
		DecompInterface ifc = new DecompInterface();
		ifc.openProgram(currentProgram);
		int n = 0;
		try (PrintWriter out = new PrintWriter(args[0])) {
			FunctionIterator it = currentProgram.getFunctionManager().getFunctions(true);
			while (it.hasNext() && n < max && !monitor.isCancelled()) {
				Function f = it.next();
				String full = f.getName(true);
				if (!p.matcher(full).find()) {
					continue;
				}
				out.println("// ==== " + f.getEntryPoint() + " " + full + " size=" + f.getBody().getNumAddresses());
				DecompileResults r = ifc.decompileFunction(f, 60, monitor);
				out.println(r.decompileCompleted() ? r.getDecompiledFunction().getC()
						: "// decompile failed: " + r.getErrorMessage());
				n++;
			}
		}
		println("DecompileMatching done: " + n + " functions");
	}
}
