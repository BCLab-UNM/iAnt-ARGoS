from xml.dom.minidom import parse, parseString
import random
import subprocess
#import sys

class Random_Argos:

  def __init__(self, argos_xml = None):
    self.argos_xml = argos_xml

  # Updates the ARGoS XML file's random seed.
  def update_random_seed(self):

    ## looks through the xml file

      xml = parse(self.argos_xml)

      #looks at the variables under the experiment tag in the argos XML file.
      framework = xml.getElementsByTagName("experiment")

      attrs = framework[0]


      attrs.setAttribute('random_seed', str(random.randint(1, 1000000)))
      #attrs.setAttribute('length', 360)




#####
      ## Sets parameters in the command line when the function is called.
      # attrs.setAttribute(sys.argv[1], sys.argv[2])
      # name, value

####
      xml.writexml(open(self.argos_xml, 'w'))




if __name__ == "__main__":

  this_run = Random_Argos("./experiments/iAnt_macPower.argos")

  this_run.update_random_seed()

  output = subprocess.check_output(['argos3 -n -c ' + this_run.argos_xml], shell=True, stderr=subprocess.STDOUT)
  
  print output