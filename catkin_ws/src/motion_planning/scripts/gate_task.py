from combination_tasks import MasterSlaveTask
from condition_task import ConditionTaskBase
from task import Task

class GateTask(Task):
    
    def __init__ (self):
        super(GateTask, self).__init__(*args, **kwargs)

    def _on_task_run(self):
        self.ifFoundGate =
        self.master = ConditionTaskBase(self.ifFoundGate)
        self.conditionBody = MasterSlaveTask(self.master, self.slave) 
        
    
