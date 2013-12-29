import java.io.*;

class PBXGen { 
    PBXGen() throws Exception {
        main(new String[] {"../../Make/B-Human.xcodeproj/project.pbxgen", "../../Make/B-Human.xcodeproj/project.pbxproj"});
    }
    
    public static void main(String[] args) throws Exception {
        String rulesFile = null;
        String projectFile = null;
        boolean showSymbols = false;
        boolean showSource = false;

        for (int i = 0; i < args.length; ++i) {
            String arg = args[i];
            if (arg.equals("-h")) {
                usage();
                return;
            } else if (arg.equals("-l")) {
                showSymbols = true;
            } else if (arg.equals("-s")) {
                showSource = true;
            } else if (arg.length() > 0 && arg.charAt(0) == '-') {
                System.out.println("Unknown option " + arg);
                usage();
                return;
            } else if (projectFile != null) {
                System.out.println("Only two filenames allowed");
                usage();
                return;
            } else if (rulesFile != null) {
                projectFile = arg;
            } else {
                rulesFile = arg;
            }
        }
            
        if (projectFile == null) {
            System.out.println("Two files required");
            usage();
            return;
        }
        
        SyntaxAnalysis s1 = new SyntaxAnalysis(rulesFile, showSymbols, showSource);
        SyntaxAnalysis s2 = new SyntaxAnalysis(projectFile, showSymbols, showSource);
        Project p = new Project(s1.parse(), s2.parse());
        s1.close();
        s2.close();
        
        if(p.update()) {
            TreeStream stream = new TreeStream(
                    new PrintStream(new FileOutputStream(projectFile + ".tmp")));
            p.print(stream);
            stream.close();
        }
    }
    
    private static void usage() {
        System.out.println("java -jar PBXgen.jar [-h] [-l] [-s] <rules> <project>");
        System.out.println("    -h       Show this help");
        System.out.println("    -l       Show result of lexical analysis");
        System.out.println("    -s       Show source file");
    }
}
