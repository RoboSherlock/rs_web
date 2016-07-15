import cmd
import os

class RSQuery(cmd.Cmd):
    """RoboSherlock Query Console
    Send a service call to detect specific entities in RoboSherlock"""
    prompt = "rs_query$ "
    
    SOLUTION_COMMANDS = [ 'all', 'single' ]
    DETECT_COMMANDS = [ 'SHAPE', 'COLOR', 'SIZE', 'LOCATION', 'LOGO', 'TEXT', 'PRODUCT', 'DETECTION' ]
    # OBJECT_NAMES = [ 'http://knowrob.org/kb/rs_components.owl#MondaminPancakeMix', 'http://knowrob.org/kb/rs_components.owl#PancakeMaker', 'http://knowrob.org/kb/rs_components.owl#Milk', 'http://knowrob.org/kb/rs_components.owl#SomatTabs']
    OBJECT_NAMES = [ 'MondaminPancakeMix', 'PancakeMaker', 'Milk', 'SomatTabs']
    OBJECT_NAMESPACE_PREFIX = "http://knowrob.org/kb/rs_objects.owl#"

    # Take the first value in SOLUTION_COMMANDS as default0
    solution_mode = SOLUTION_COMMANDS[0]

    def map_solution_mode_to_service_name(self):
      if self.solution_mode == 'all':
        return "/RoboSherlock_robosherlock/designator_request/all_solutions"
      elif self.solution_mode == 'single':
        return "/RoboSherlock_robosherlock/designator_request/single_solution"

      return "FAIL"

    def description_template(self,idx,key,value_string=""):
        desc_str = """
    - id: """+str(idx)+"""
      parent: 0
      type: 0
      key: '"""+key+"""'
      value_string: '"""+value_string+"""'
      value_float: 0.0
      value_data: ''
      value_posestamped:
        header:
          seq: 0                    
          stamp: {secs: 0, nsecs: 0}
          frame_id: ''
        pose:                               
          position: {x: 0.0, y: 0.0, z: 0.0}
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
      value_pose:                         
        position: {x: 0.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        """
        return desc_str


    def do_solution(self, command):
      """Set the solution mode. If RoboSherlock finds multiple solutions
      that could detect the desired predicates, should it execute them?
      Possible values are: all, single.

      Example call: solution single"""
      if not command: 
        print "Command empty?"
        return

      if command in self.SOLUTION_COMMANDS:
        self.solution_mode = command
      else:
        print "Unknown mode. Use 'help solution' to check out which modes are available"
        return

    def complete_solution(self, text, line, begidx, endidx):
        if not text:
            completions = self.SOLUTION_COMMANDS[:]
        else:
            completions = [ c
                            for c in self.SOLUTION_COMMANDS
                            if c.startswith(text)
                            ]
        return completions

    def do_detectobj(self, command):
        "Query RoboSherlock and ask it to detect an specific object in the ontology. You do NOT need to add the Ontology namespace prefix here. This will be done automatically."

        if not command: 
            print "Command empty?"
            return

        call_str = """rosservice call """ + self.map_solution_mode_to_service_name() + """ \"request:
  designator:
    type: 0
    description:"""
        call_str+=self.description_template(1,"TYPE",self.OBJECT_NAMESPACE_PREFIX+command)

        call_str+="\""
        output = os.popen(call_str).read()
        print output

    def complete_detectobj(self, text, line, begidx, endidx):
        if not text:
            completions = self.OBJECT_NAMES[:]
        else:
            completions = [ f
                    for f in self.OBJECT_NAMES
                    if f.startswith(text)
                    ]
        return completions

    def do_detect(self, command):
        "Query RoboSherlock and ask it to detect an object by some predefined predicates"

        if not command: 
            print "Command empty?"
            return

        call_str = """rosservice call """ + self.map_solution_mode_to_service_name() + """ \"request:
  designator:
    type: 0
    description:"""
        # call_str+=self.description_template(1,"shape")

        idx = 1
        for pred in command.split(","):
            if pred not in self.DETECT_COMMANDS:
                print "Unknown predicate given: " + pred
                return
            call_str += self.description_template(idx,pred)
            idx += 1

        call_str+="\""
        output = os.popen(call_str).read()
        print output
            # print output
        # else:
        #     print "Unknown command"
    
    def complete_detect(self, text, line, begidx, endidx):
        last_parameter = ""
        # There is already a comma in the parameters. Only consider everything that's right of it.
        if "," in text:
            last_parameter = text.rsplit(',',1)[1] 
        else:
            last_parameter = text

        if not text:
            completions = self.DETECT_COMMANDS[:]
        else:
            completions = [ f
                            for f in self.DETECT_COMMANDS
                            if f.startswith(last_parameter)
                            ]
        return completions
    
    def do_EOF(self, line):
        print ""
        return True

    # Don't redo the last command, if the line is empty.
    def emptyline(self):
        pass

if __name__ == '__main__':
    print "** Welcome to rs_detect.\n   Note: Please use commas to seperate predicates in the detect command."
    RSQuery().cmdloop()
