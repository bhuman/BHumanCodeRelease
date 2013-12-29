import java.io.*;
import java.util.*;

class FileSet extends TreeSet<String>{
    private String[] includeFilters;
    private String[] excludeFilters;
    private String includeFilter;
    
    FileSet(String[] includes, String[] excludes, String[] statics) {
        includeFilters = convert(includes);
        excludeFilters = convert(excludes);
        for (int i = 0; i < includes.length; ++i) {
            includeFilter = includeFilters[i];
            File file = new File(includes[i]).getParentFile();
            collect(file, file.getParent());
        }
        for (String s : statics) {
             add(s);
        }
    }
    
    private String[] convert(String[] input) {
        String[] output = new String[input.length];
        for (int i = 0; i < input.length; ++i) {
            output[i] = input[i].replaceAll("\\*\\*", ":")
                .replaceAll("\\*", "[^/]*")
                .replaceAll("\\.", "\\\\.")
                .replaceAll(":", ".*");
        }
        return output;
    }
    
    private void collect(File file, String path) {
        if (!file.getName().startsWith(".")) {
            String fullPath = path + "/" + file.getName();
            label: if(file.isDirectory()) {
                File[] files = file.listFiles();
                for (File f : files) {
                    collect(f, fullPath);
                }
            } else if (fullPath.matches(includeFilter)) {
                for (String e : excludeFilters) {
                    if (fullPath.matches(e)) {
                        break label;
                    }
                }
                add(fullPath);
            }
        }
    }
    
    boolean matches(String fullPath) {
        for (String i : includeFilters) {
            if (fullPath.matches(i)) {
                for (String e : excludeFilters) {
                    if (fullPath.matches(e)) {
                        return false;
                    }
                }
                return true;
            }
        }
        return false;
    }
    
    void print() {
        for (String s : this) {
            System.out.println(s);
        }
    }
}
