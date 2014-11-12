#include <Python.h>
#include "structmember.h"

#include <rcl/rcl.h>

typedef struct {
  PyObject_HEAD
  /* Type-specific fields go here. */
  PyObject * anyexecutable_handle;
  PyObject * subscription_handle;
} rcl_api_AnyExecObject;

static void
rcl_api_AnyExecObject_dealloc(rcl_api_AnyExecObject* self)
{
  Py_XDECREF(self->anyexecutable_handle);
  Py_XDECREF(self->subscription_handle);
  Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
rcl_api_AnyExecObject_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
  rcl_api_AnyExecObject *self;

  self = (rcl_api_AnyExecObject*)type->tp_alloc(type, 0);
  if (self != NULL)
    {
      self->anyexecutable_handle = Py_None;
      self->subscription_handle = Py_None;
    }

  return (PyObject*)self;
}

static int
rcl_api_AnyExecObject_init(rcl_api_AnyExecObject *self, PyObject *args)
{
  PyObject *anyexecutable_handle = NULL, *subscription_handle = NULL, *tmp;

  if (!PyArg_ParseTuple(args, "OO", &anyexecutable_handle,
                        &subscription_handle))
    {
      return -1;
    }

  tmp = self->anyexecutable_handle;
  Py_INCREF(anyexecutable_handle);
  self->anyexecutable_handle = anyexecutable_handle;
  Py_XDECREF(tmp);

  tmp = self->subscription_handle;
  Py_INCREF(subscription_handle);
  self->subscription_handle = subscription_handle;
  Py_XDECREF(tmp);

  return 0;
}

static PyMemberDef rcl_api_AnyExecObject_members[] = {
  { "anyexecutable_handle", T_OBJECT_EX,
    offsetof(rcl_api_AnyExecObject, anyexecutable_handle),
    0, "anyexecutable handle" },
  { "subscription_handle", T_OBJECT_EX,
    offsetof(rcl_api_AnyExecObject, subscription_handle),
    0, "subscription handle" },
  { NULL }  /* Sentinel */
};

static PyObject *
rcl_api_AnyExecObject_getsubscription_handle(rcl_api_AnyExecObject *self,
                                             void *closure)
{
  Py_INCREF(self->subscription_handle);
  return self->subscription_handle;
}

static int
rcl_api_AnyExecObject_setanyexecutable_handle(rcl_api_AnyExecObject *self,
                                              PyObject *value, void *closure)
{
  PyErr_SetString(PyExc_TypeError,
                  "Cannot set the anyexecutable_handle attribute");
  return -1;
}


static PyObject *
rcl_api_AnyExecObject_getanyexecutable_handle(rcl_api_AnyExecObject *self,
                                              void *closure)
{
  Py_INCREF(self->anyexecutable_handle);
  return self->anyexecutable_handle;
}

static int
rcl_api_AnyExecObject_setsubscription_handle(rcl_api_AnyExecObject *self,
                                             PyObject *value, void *closure)
{
  PyErr_SetString(PyExc_TypeError,
                  "Cannot set the subscription_handle attribute");
  return -1;
}

static PyGetSetDef rcl_api_AnyExecObject_accessors[] = {
  { "anyexecutable_handle",
    (getter)rcl_api_AnyExecObject_getanyexecutable_handle,
    (setter)rcl_api_AnyExecObject_setanyexecutable_handle,
    "anyexecutable handle", NULL },
  { "subscription_handle",
    (getter)rcl_api_AnyExecObject_getsubscription_handle,
    (setter)rcl_api_AnyExecObject_setsubscription_handle,
    "subscription handle", NULL },
  { NULL }  /* Sentinel */
};

static PyTypeObject rcl_api_AnyExecType = {
  PyVarObject_HEAD_INIT(NULL, 0)
  "rcl_api.AnyExec",                           /* tp_name */
  sizeof(rcl_api_AnyExecObject),               /* tp_basicsize */
  0,                                           /* tp_itemsize */
  (destructor)rcl_api_AnyExecObject_dealloc,   /* tp_dealloc */
  0,                                           /* tp_print */
  0,                                           /* tp_getattr */
  0,                                           /* tp_setattr */
  0,                                           /* tp_reserved */
  0,                                           /* tp_repr */
  0,                                           /* tp_as_number */
  0,                                           /* tp_as_sequence */
  0,                                           /* tp_as_mapping */
  0,                                           /* tp_hash  */
  0,                                           /* tp_call */
  0,                                           /* tp_str */
  0,                                           /* tp_getattro */
  0,                                           /* tp_setattro */
  0,                                           /* tp_as_buffer */
  Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,    /* tp_flags */
  "AnyExec objects",                           /* tp_doc */
  0,                                           /* tp_traverse */
  0,                                           /* tp_clear */
  0,                                           /* tp_richcompare */
  0,                                           /* tp_weaklistoffset */
  0,                                           /* tp_iter */
  0,                                           /* tp_iternext */
  0,                                           /* tp_methods */
  rcl_api_AnyExecObject_members,               /* tp_members */
  rcl_api_AnyExecObject_accessors,             /* tp_getset */
  0,                                           /* tp_base */
  0,                                           /* tp_dict */
  0,                                           /* tp_descr_get */
  0,                                           /* tp_descr_set */
  0,                                           /* tp_dictoffset */
  (initproc)rcl_api_AnyExecObject_init,        /* tp_init */
  0,                                           /* tp_alloc */
  rcl_api_AnyExecObject_new,                   /* tp_new */
};


static void rcl_node_t_ptr_destructor(PyObject * pynode)
{
  rcl_node_t * cnode = (rcl_node_t*)PyCapsule_GetPointer(
    pynode, "rcl_node_t_ptr");

  rcl_destroy_node(cnode);
}

static PyObject * executor_add_node(PyObject * self, PyObject * args)
{
  Py_RETURN_NONE;
}

static PyObject * executor_get_next_executable(PyObject * self,
                                               PyObject * args)
{
  rcl_any_executable_t * rcl_any_executable =
    (rcl_any_executable_t*)malloc(sizeof(rcl_any_executable_t));

  rcl_executor_helper_t * executor_helper = rcl_create_executor_helper();

  bool non_blocking = false;

  rcl_get_next_any_executable(executor_helper, rcl_any_executable,
                              non_blocking);

  PyObject *pyrcl_any_executable = PyCapsule_New(
    (void*)rcl_any_executable, "rcl_any_executable_t_ptr", NULL);

  PyObject *pyrcl_subscription = Py_None;

  if (rcl_any_executable->subscription_info != NULL &&
      *(rcl_any_executable->subscription_info) != NULL)
    {
      pyrcl_subscription = PyCapsule_New(
        (void*)*(rcl_any_executable->subscription_info),
        "rcl_subscription_t_ptr", NULL);
    }
  PyObject *args_list = Py_BuildValue("OO", pyrcl_any_executable,
                                      pyrcl_subscription);
  PyObject *obj = PyObject_CallObject((PyObject*)&rcl_api_AnyExecType,
                                      args_list);
  Py_DECREF(args_list);

  return (PyObject*)obj;
}

static PyObject * executor_create(PyObject * self, PyObject * args)
{
  rcl_executor_helper_t * cexecutor = rcl_create_executor_helper();
  PyObject * ret = PyCapsule_New(
    (void*)cexecutor, "rcl_executor_helper_t_ptr", NULL);

  Py_INCREF(ret);
  return ret;
}

static PyObject * executor_execute_any_executable(
  PyObject * self, PyObject * args)
{
  Py_RETURN_NONE;
}

static PyObject * node_create(PyObject * self, PyObject * args)
{
  const char * name;

  if (!PyArg_ParseTuple(args, "s", &name))
    {
      return NULL;
    }
  rcl_node_t * cnode = rcl_create_node(name);
  PyObject * ret = PyCapsule_New((void*)cnode, "rcl_node_t_ptr",
                                 &rcl_node_t_ptr_destructor);
  Py_INCREF(ret);
  return ret;
}

static PyObject * subscription_create(PyObject * self, PyObject * args)
{
  PyObject * pynode;
  PyObject * pydata_type;
  const char * topic_name;
  size_t queue_size;

  if (!PyArg_ParseTuple(args, "OOsI", &pynode, &pydata_type, &topic_name,
                        &queue_size))
    {
      return NULL;
    }

  PyObject * pytype_support = PyObject_GetAttrString(
    pydata_type, "TYPE_SUPPORT");

  rcl_node_t * cnode = (rcl_node_t*)PyCapsule_GetPointer(
    pynode, "rcl_node_t_ptr");

  rosidl_message_type_support_t * ctype_support =
    (rosidl_message_type_support_t*)PyCapsule_GetPointer(
      pytype_support, "rosidl_message_type_support_t_ptr");
  rcl_subscription_t * csubscription = rcl_create_subscription(
    cnode, ctype_support, topic_name, queue_size);
  assert(csubscription != NULL);
  PyObject * ret = PyCapsule_New(
    (void*)csubscription, "rcl_subscription_t_ptr", NULL);
  Py_INCREF(ret);
  return ret;
}

static PyObject * subscription_take(
  PyObject * self, PyObject * pysubscription)
{
  rcl_subscription_t * csubscription = PyCapsule_GetPointer(
    pysubscription, "rcl_subscription_t_ptr");

  void * ros_message;
  rcl_ret_t ret = rcl_take(csubscription, &ros_message);

  // TODO: convert ros_message to a Py/C message
  Py_RETURN_NONE;
}

static PyMethodDef rcl_api_methods[] = {
  { "executor_add_node", executor_add_node, METH_VARARGS,
    "Add a node to the executor." },
  { "executor_create", executor_create, METH_VARARGS,
    "Create an executor." },
  { "executor_execute_any_executable", executor_execute_any_executable,
    METH_VARARGS, "Execute an executable." },
  { "executor_get_next_executable", executor_get_next_executable, METH_NOARGS,
    "Obtain the next executable." },
  { "node_create", node_create, METH_VARARGS,
    "Create a node." },
  { "subscription_create", subscription_create, METH_VARARGS,
    "Create a subscription." },
  { "subscription_take", subscription_take, METH_O,
    "Retrieve a message from a subscription." },
  { NULL, NULL, 0, NULL }
};

static struct PyModuleDef rcl_api = {
  PyModuleDef_HEAD_INIT, "rcl_api", NULL, -1, rcl_api_methods
};

PyMODINIT_FUNC PyInit_rcl_api(void)
{
  // rcl_init();

  PyObject* m;

  rcl_api_AnyExecType.tp_new = PyType_GenericNew;
  if (PyType_Ready(&rcl_api_AnyExecType) < 0)
    return NULL;

  m = PyModule_Create(&rcl_api);
  if (m == NULL)
    return NULL;

  Py_INCREF(&rcl_api_AnyExecType);
  PyModule_AddObject(m, "AnyExec", (PyObject*)&rcl_api_AnyExecType);
  return m;
}
