import dependency_injector.providers as providers
from combination_tasks import ListTask

task_provider = providers.Singleton(ListTask)
task_provider2 = providers.Singleton(ListTask)

print(task_provider([]))
print(task_provider2([]))
