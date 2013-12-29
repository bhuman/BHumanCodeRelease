import java.util.*;
import java.io.*;
import java.math.*;

class Project {
    private static final Attribute isaFileReference = new Attribute(new Literal("isa"), new Literal("PBXFileReference"));
    private static final Attribute isaGroup = new Attribute(new Literal("isa"), new Literal("PBXGroup"));
    private static final Attribute isaBuildFile = new Attribute(new Literal("isa"), new Literal("PBXBuildFile"));
    private static final Attribute fileEncoding = new Attribute(new Literal("fileEncoding"), new Literal("4"));
    private static final Attribute indentWidth4 = new Attribute(new Literal("indentWidth"), new Literal("4")); 
    private static final Attribute fileTypeCpp = new Attribute(new Literal("lastKnownFileType"), new Literal("sourcecode.cpp.cpp"));
    private static final Attribute fileTypeC = new Attribute(new Literal("lastKnownFileType"), new Literal("sourcecode.c.c"));
    private static final Attribute fileTypeH = new Attribute(new Literal("lastKnownFileType"), new Literal("sourcecode.cpp.h"));
    private static final Attribute fileTypeHtml = new Attribute(new Literal("lastKnownFileType"), new Literal("text.html"));
    private static final Attribute fileTypeCss = new Attribute(new Literal("lastKnownFileType"), new Literal("text.css"));
    private static final Attribute fileTypeXml = new Attribute(new Literal("lastKnownFileType"), new Literal("text.xml"));
    private static final Attribute fileTypeBmp = new Attribute(new Literal("lastKnownFileType"), new Literal("image.bmp"));
    private static final Attribute fileTypeIcns = new Attribute(new Literal("lastKnownFileType"), new Literal("image.icns"));
    private static final Attribute fileTypePng = new Attribute(new Literal("lastKnownFileType"), new Literal("image.png"));
    private static final Attribute fileTypeText = new Attribute(new Literal("lastKnownFileType"), new Literal("text"));
    private static final Attribute fileTypeObjc = new Attribute(new Literal("lastKnownFileType"), new Literal("sourcecode.cpp.objc"));
    private static final Attribute fileTypeObjcpp = new Attribute(new Literal("lastKnownFileType"), new Literal("sourcecode.cpp.objcpp"));
    private static final Attribute fileTypeJava = new Attribute(new Literal("lastKnownFileType"), new Literal("sourcecode.java"));
        
    private static final Attribute sourceTree = new Attribute(new Literal("sourceTree"), new Literal("SOURCE_ROOT"));

    private static final List ROOT = new List();
    
    private Record rules;
    private Record root;
    private Record objects;
    private HashMap<String, String> pathToId = new HashMap<String, String>();
    private HashMap<String, String> idToPath = new HashMap<String, String>();
    private HashSet<String> newIds = new HashSet<String>();
    private boolean modified;
    
    Project(Record rules, Record root) {
        this.rules = rules;
        this.root = root;
        objects = (Record) root.get("objects");
    }
    
    void print(TreeStream stream) {
        stream.println("// !$*UTF8*$!");
        root.print(stream);
        stream.println();
    }
    
    boolean update() throws Exception {
        modified = false;

        collectAndRemove(getSection("PBXGroup"), null);
        
        for(String attr : new String[] {"inputPaths", "outputPaths"}) {
            Record r = (Record) rules.get(attr);
            if (r != null) {
                for (Value v : r.values) {
                    Attribute a = (Attribute) v;
                    r = (Record) a.value;
                    String name = removeQuotes(a.key.literal);
                    String anchor = removeQuotes(((Literal) r.get("anchor")).literal);
                    String[] includes = getPatterns(r, "includes");
                    String[] excludes = getPatterns(r, "excludes");
                    String[] statics = getPatterns(r, "statics");
                    updateFileList(name, anchor, attr, new FileSet(includes, excludes, statics));
                }
            }
        }
        
        Record r = (Record) rules.get("sources");
        if (r != null) {
            for (Value v : r.values) {
                Attribute a = (Attribute) v;
                r = (Record) a.value;
                String name = removeQuotes(a.key.literal);
                String root = removeQuotes(((Literal) r.get("root")).literal);
                String[] includes = getPatterns(r, "includes");
                String[] excludes = getPatterns(r, "excludes");
                String[] statics = getPatterns(r, "statics");
                updateSources(name, root, new FileSet(includes, excludes, statics));
            }
        }
        sort(getSection("PBXFileReference"));
        sort(getSection("PBXGroup"));

        for (Map.Entry<String, String> e : pathToId.entrySet()) {
            idToPath.put(e.getValue(), e.getKey());
        }
        
        r = (Record) rules.get("buildFiles");
        if (r != null) {
            for (Value v : r.values) {
                Attribute a = (Attribute) v;
                r = (Record) a.value;
                String name = removeQuotes(a.key.literal);
                String anchor = removeQuotes(((Literal) r.get("anchor")).literal);
                String[] includes = getPatterns(r, "includes");
                String[] excludes = getPatterns(r, "excludes");
                String[] first = getPatterns(r, "first");
                String[] last = getPatterns(r, "last");
                String[] statics = getPatterns(r, "statics");
                updateBuildFiles(name, anchor, new FileSet(includes, excludes, statics), first, last);
            }
        }

        java.util.List<Value> section = getSection("PBXBuildFile");
        if (section != null) {
            sort(section);
        }
        
        return modified;
    }

    private String[] getPatterns(Record r, String key) {
        List list = ((List) r.get(key));
        if (list == null) {
            return new String[0];
        } else {
            String[] patterns = new String[list.literals.size()];
            int i = 0;
            for (Literal l : list.literals) {
                patterns[i++] = removeQuotes(l.literal);
            }
            return patterns;
        }
    }
    
    private String removeQuotes(String s) {
        if (s.startsWith("\"")) {
            s = s.substring(1);
        }
        if (s.endsWith("\"")) {
            s = s. substring(0, s.length() - 1);
        }
        return s;
    }
    
    private void updateFileList(String name, String id, String attr, FileSet files) {
        List oldList = (List) ((Record) objects.get(id)).get(attr);
        LinkedList<Literal> newList = new LinkedList<Literal>();
        for (String s : files) {
            newList.add(new Literal(s));
        }
        if(!oldList.literals.equals(newList)) {
            System.out.println("Updating Xcode " + name + " Files");
            oldList.literals = newList;
            modified = true;
        }
    }
    
    private void updateSources(String name, String rootPath, FileSet files) {
        java.util.List<Value> section = getSection("PBXFileReference");
        int found = collectAndRemove(section, files);
        int reused = 0;
        
        for (String path : files) {
            File file = new File(path);

            Record r = new Record();
            r.values.add(isaFileReference);
            if (path.endsWith(".cpp") || path.endsWith(".cc")) {
                r.values.add(fileEncoding);
                r.values.add(fileTypeCpp);
            } else if (path.endsWith(".c")) {
                r.values.add(fileEncoding);
                r.values.add(fileTypeC);
            } else if (path.endsWith(".h") || path.endsWith(".hh")) {
                r.values.add(fileEncoding);
                r.values.add(fileTypeH);
            } else if (path.endsWith(".htm") || path.endsWith(".html")) {
                r.values.add(fileEncoding);
                r.values.add(fileTypeHtml);
            } else if (path.endsWith(".css")) {
                r.values.add(fileEncoding);
                r.values.add(fileTypeCss);
            } else if (path.endsWith(".qrc") || path.endsWith(".qhp") || path.endsWith(".qhcp") || path.endsWith(".xsd")) {
                r.values.add(fileEncoding);
                r.values.add(fileTypeXml);
            } else if (path.endsWith(".bmp")) {
                r.values.add(fileTypeBmp);
            } else if (path.endsWith(".icns")) {
                r.values.add(fileTypeIcns);
            } else if (path.endsWith(".png")) {
                r.values.add(fileTypePng);
            } else if (path.endsWith(".m")) {
                r.values.add(fileTypeObjc);
            } else if (path.endsWith(".mm")) {
                r.values.add(fileTypeObjcpp);
            } else if (path.endsWith(".java")) {
                r.values.add(fileTypeJava);
                r.values.add(indentWidth4); 
            } else {
                r.values.add(fileEncoding);
                r.values.add(fileTypeText);
            }
            r.values.add(new Attribute(new Literal("name"), new Literal(file.getName())));
            r.values.add(new Attribute(new Literal("path"), new Literal(path)));
            r.values.add(sourceTree);

            String id = pathToId.get(path);
            if (id == null) {
                id = newId(path);
                pathToId.put(path, id);
                ++found;
            } else {
                ++reused;
            }
            section.add(new Attribute(new Literal(id, "/* " + file.getName() + " */"), r));
        }
        
        updateGroups(rootPath, files, pathToId);
        
        if (found != reused) {
            System.out.println("Updating Xcode " + name + " Source Files");
            modified = true;
        }
    }
    
    private void updateBuildFiles(String name, String id, FileSet files, String[] first, String[] last) throws Exception {
        java.util.List<Value> section = getSection("PBXBuildFile");
        List list = (List) ((Record) objects.get(id)).get("files");
        HashSet<String> ids = new HashSet<String>();
        for (Literal l : list.literals) {
            ids.add(l.literal);
        }
        list.literals.clear();
 
        HashMap<String, String> pathToBuild = collectAndRemoveBuild(section, ids);

        TreeSet<Literal> buildFiles = new TreeSet<Literal>();
        LinkedList<Literal> lastFiles = new LinkedList<Literal>();
        int reused = 0;
        loop: for (String path : files) {
            File file = new File(path);

            Record r = new Record();
            r.values.add(isaBuildFile);
            r.values.add(new Attribute(new Literal("fileRef"), new Literal(pathToId.get(path), "/* " + file.getName() + " */")));

            id = pathToBuild.get(path);
            if (id == null) {
                id = newId(path + "#" + name);
                pathToBuild.put(path, id);
            } else {
                ++reused;
            }
            Literal l = new Literal(id, "/* " + file.getName() + " in Sources */");
            section.add(new Attribute(l, r));
            if (buildFiles.contains(l)) {
                String s = l.comment.substring(3);
                s = s.substring(0, s.indexOf(' '));
                throw new Exception("Error when updating Xcode " + name + ": " + s);
            }
            
            for (String s : first) {
                if (s.equals(path)) {
                    list.literals.add(l);
                    continue loop;
                }
            }

            for (String s : last) {
                if (s.equals(path)) {
                    lastFiles.add(l);
                    continue loop;
                }
            }

            buildFiles.add(l);
        }

        
        for (Literal l : buildFiles) {
            list.literals.add(l);
        }
        
        for (Literal l : lastFiles) {
            list.literals.add(l);
        }
        
        if (pathToBuild.size() != reused) {
            System.out.println("Updating Xcode " + name + " Build Files");
            modified = true;
        }
    }
    
    private int collectAndRemove(java.util.List<Value> section, FileSet mask) {
        Iterator<Value> i = section.iterator();
        int found = 0;
        while (i.hasNext()) {
            Attribute a = (Attribute) i.next();
            Record r = (Record) a.value;
            Literal l = (Literal) r.get("path");
            String path = l != null ? removeQuotes(l.literal) : null;
            if (path != null && (mask == null || mask.matches(path))) {
                pathToId.put(removeQuotes(path), ((Literal) a.key).literal); 
                i.remove();
                ++found;
            }
        }
        return found;
    }

    private HashMap<String, String> collectAndRemoveBuild(java.util.List<Value> section, HashSet<String> ids) {
        HashMap<String, String> pathToBuild = new HashMap<String, String>();
        Iterator<Value> i = section.iterator();
        while (i.hasNext()) {
            Attribute a = (Attribute) i.next();
            if (ids.contains(a.key.literal)) {
                Record r = (Record) a.value;
                String id = ((Literal) r.get("fileRef")).literal;
                String path = idToPath.get(id);
                pathToBuild.put(path, a.key.literal);
                 i.remove();
            }
        }
        return pathToBuild;
    }

    private void updateGroups(String rootPath, FileSet files, HashMap<String, String> pathToId) {
        java.util.List<Value> section = getSection("PBXGroup");
        HashMap<String, List> pathToGroup = new HashMap<String, List>();
        pathToGroup.put(rootPath, ROOT);
        for (String path : files) {
            createPath(section, pathToGroup, path, true);
        }
        
        
    }
    
    private void createPath(java.util.List<Value> section, HashMap<String, List> pathToGroup, 
            String path, boolean isFile) {
        File file = new File(path);
        String parent = file.getParent();
        List l = pathToGroup.get(parent);
        if (l == null) {
            createPath(section, pathToGroup, parent, false);
            l = pathToGroup.get(parent);
        }
        
        String id = pathToId.get(path);
        if (!isFile) {
            if (id == null) {
                id = newId(path);
                pathToId.put(path, id);
            }
            
            List children = new List();
            pathToGroup.put(path, children);

            Record r = new Record();
            r.multiLine = true;
            r.values.add(isaGroup);
            r.values.add(new Attribute(new Literal("children"), children));
            r.values.add(new Attribute(new Literal("name"), new Literal(file.getName())));
            r.values.add(new Attribute(new Literal("path"), new Literal(path)));
            r.values.add(sourceTree);
            section.add(new Attribute(new Literal(id, "/* " + file.getName() + " */"), r));
        }
        
        if (l != ROOT) {
            l.literals.add(new Literal(id, "/* " + file.getName() + " */"));
        }
    }    
    
    private String newId(String path) {
        BigInteger sum = BigInteger.ZERO;
        for(int i = 0; i < path.length(); ++i) {
            sum = sum.xor(sum.shiftRight(8)).shiftLeft(1).add(new BigInteger("" + (int) path.charAt(i)));
        }
        String id = "000000000000000000000000" + sum.toString();
        id = id.substring(id.length() - 24, id.length());
        while (newIds.contains(id)) {
            id = id.substring(23) + id.substring(0, 23);
        }
        newIds.add(id);
        return id;
    }
    
    private java.util.List<Value> getSection(String name) {
        int index = 0;
        int start = 0;
        for (Value v : objects.values) {
            if (v instanceof Comment) {
                Comment c = (Comment) v;
                if (start == 0) {
                    start = index + 1;
                } else {
                    String s = c.comment.substring(7);
                    s = s.substring(0, s.indexOf(' '));
                    if (s.equals(name)) { 
                        return objects.values.subList(start, index);
                    }
                    start = 0;
                }
            }
            ++index;
        }
        return null;
    }
    
    private void sort(java.util.List<Value> section) {
        TreeSet<Attribute> sources = new TreeSet<Attribute>();
        for (Value v : section) {
            sources.add((Attribute) v);
        }
        section.clear();
        for (Attribute a : sources) {
            section.add(a);
        }
    }
}
