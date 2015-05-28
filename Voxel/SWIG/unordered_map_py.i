/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 *
 * Modified code from SWIG, to make it work for std::unordered_map
 */

%fragment("StdMapTraits","header",fragment="StdSequenceTraits")
{
  namespace swig {
    template <class SwigPySeq, class K, class T >
    inline void
    assign(const SwigPySeq& swigpyseq, std::unordered_map<K,T > *unordered_map) {
      typedef typename std::unordered_map<K,T>::value_type value_type;
      typename SwigPySeq::const_iterator it = swigpyseq.begin();
      for (;it != swigpyseq.end(); ++it) {
        unordered_map->insert(value_type(it->first, it->second));
      }
    }

    template <class K, class T>
    struct traits_asptr<std::unordered_map<K,T> >  {
      typedef std::unordered_map<K,T> unordered_map_type;
      static int asptr(PyObject *obj, unordered_map_type **val) {
        int res = SWIG_ERROR;
        if (PyDict_Check(obj)) {
          SwigVar_PyObject items = PyObject_CallMethod(obj,(char *)"items",NULL);
%#if PY_VERSION_HEX >= 0x03000000
          /* In Python 3.x the ".items()" method return a dict_items object */
          items = PySequence_Fast(items, ".items() havn't returned a sequence!");
%#endif
          res = traits_asptr_stdseq<std::unordered_map<K,T>, std::pair<K, T> >::asptr(items, val);
        } else {
          unordered_map_type *p;
          res = SWIG_ConvertPtr(obj,(void**)&p,swig::type_info<unordered_map_type>(),0);
          if (SWIG_IsOK(res) && val)  *val = p;
        }
        return res;
      }      
    };
      
    template <class K, class T >
    struct traits_from<std::unordered_map<K,T> >  {
      typedef std::unordered_map<K,T> unordered_map_type;
      typedef typename unordered_map_type::const_iterator const_iterator;
      typedef typename unordered_map_type::size_type size_type;
            
      static PyObject *from(const unordered_map_type& unordered_map) {
#ifdef SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS
        swig_type_info *desc = swig::type_info<unordered_map_type>();
        if (desc && desc->clientdata) {
          return SWIG_NewPointerObj(new unordered_map_type(unordered_map), desc, SWIG_POINTER_OWN);
        }
#endif
        size_type size = unordered_map.size();
        int pysize = (size <= (size_type) INT_MAX) ? (int) size : -1;
        if (pysize < 0) {
          SWIG_PYTHON_THREAD_BEGIN_BLOCK;
          PyErr_SetString(PyExc_OverflowError,
                          "unordered_map size not valid in python");
          SWIG_PYTHON_THREAD_END_BLOCK;
          return NULL;
        }
        PyObject *obj = PyDict_New();
        for (const_iterator i= unordered_map.begin(); i!= unordered_map.end(); ++i) {
          swig::SwigVar_PyObject key = swig::from(i->first);
          swig::SwigVar_PyObject val = swig::from(i->second);
          PyDict_SetItem(obj, key, val);
        }
        return obj;
      }
    };

    template <class ValueType>
    struct from_key_oper 
    {
      typedef const ValueType& argument_type;
      typedef  PyObject *result_type;
      result_type operator()(argument_type v) const
      {
        return swig::from(v.first);
      }
    };

    template <class ValueType>
    struct from_value_oper 
    {
      typedef const ValueType& argument_type;
      typedef  PyObject *result_type;
      result_type operator()(argument_type v) const
      {
        return swig::from(v.second);
      }
    };
    
    template<typename OutIterator, 
           typename ValueType = typename std::iterator_traits<OutIterator>::value_type,
           typename FromOper = from_oper<ValueType> >
    class SwigPyForwardIteratorOpen_T :  public SwigPyIterator_T<OutIterator>
    {
    public:
      FromOper from;
      typedef OutIterator out_iterator;
      typedef ValueType value_type;
      typedef SwigPyIterator_T<out_iterator>  base;
      typedef SwigPyForwardIteratorOpen_T<OutIterator, ValueType, FromOper> self_type;
      
      SwigPyForwardIteratorOpen_T(out_iterator curr, PyObject *seq)
        : SwigPyIterator_T<OutIterator>(curr, seq)
      {
      }
      
      PyObject *value() const {
        return from(static_cast<const value_type&>(*(base::current)));
      }
      
      SwigPyIterator *copy() const
      {
        return new self_type(*this);
      }

      SwigPyIterator *incr(size_t n = 1)
      {
        while (n--) {
          ++base::current;
        }
        return this;
      }

      SwigPyIterator *decr(size_t n = 1)
      {
        throw stop_iteration();
        return this;
      }
    };
    
    template<typename OutIterator, 
           typename ValueType = typename std::iterator_traits<OutIterator>::value_type,
           typename FromOper = from_oper<ValueType> >
    class SwigPyForwardIteratorClosed_T :  public SwigPyIterator_T<OutIterator>
    {
    public:
      FromOper from;
      typedef OutIterator out_iterator;
      typedef ValueType value_type;
      typedef SwigPyIterator_T<out_iterator>  base;    
      typedef SwigPyForwardIteratorClosed_T<OutIterator, ValueType, FromOper> self_type;
      
      SwigPyForwardIteratorClosed_T(out_iterator curr, out_iterator first, out_iterator last, PyObject *seq)
        : SwigPyIterator_T<OutIterator>(curr, seq), begin(first), end(last)
      {
      }
      
      PyObject *value() const {
        if (base::current == end) {
          throw stop_iteration();
        } else {
          return from(static_cast<const value_type&>(*(base::current)));
        }
      }
      
      SwigPyIterator *copy() const
      {
        return new self_type(*this);
      }

      SwigPyIterator *incr(size_t n = 1)
      {
        while (n--) {
          if (base::current == end) {
            throw stop_iteration();
          } else {
            ++base::current;
          }
        }
        return this;
      }

      SwigPyIterator *decr(size_t n = 1)
      {
        if(n)
          throw stop_iteration();
        return this;
      }

    private:
      out_iterator begin;
      out_iterator end;
    };

    template<class OutIterator, class FromOper, class ValueType = typename OutIterator::value_type>
    struct SwigPyMapIterator_T : SwigPyForwardIteratorClosed_T<OutIterator, ValueType, FromOper>
    {
      SwigPyMapIterator_T(OutIterator curr, OutIterator first, OutIterator last, PyObject *seq)
        : SwigPyForwardIteratorClosed_T<OutIterator,ValueType,FromOper>(curr, first, last, seq)
      {
      }
    };


    template<class OutIterator,
             class FromOper = from_key_oper<typename OutIterator::value_type> >
    struct SwigPyMapKeyIterator_T : SwigPyMapIterator_T<OutIterator, FromOper>
    {
      SwigPyMapKeyIterator_T(OutIterator curr, OutIterator first, OutIterator last, PyObject *seq)
        : SwigPyMapIterator_T<OutIterator, FromOper>(curr, first, last, seq)
      {
      }
    };
    
    template<typename OutIter>
    inline SwigPyIterator*
    make_output_forward_iterator(const OutIter& current, const OutIter& begin, const OutIter& end, PyObject *seq = 0)
    {
      return new SwigPyForwardIteratorClosed_T<OutIter>(current, begin, end, seq);
    }
    
    template<typename OutIter>
    inline SwigPyIterator*
    make_output_forward_iterator(const OutIter& current, PyObject *seq = 0)
    {
      return new SwigPyForwardIteratorOpen_T<OutIter>(current, seq);
    }

    template<typename OutIter>
    inline SwigPyIterator*
    make_output_key_iterator(const OutIter& current, const OutIter& begin, const OutIter& end, PyObject *seq = 0)
    {
      return new SwigPyMapKeyIterator_T<OutIter>(current, begin, end, seq);
    }

    template<class OutIterator,
             class FromOper = from_value_oper<typename OutIterator::value_type> >
    struct SwigPyMapValueITerator_T : SwigPyMapIterator_T<OutIterator, FromOper>
    {
      SwigPyMapValueITerator_T(OutIterator curr, OutIterator first, OutIterator last, PyObject *seq)
        : SwigPyMapIterator_T<OutIterator, FromOper>(curr, first, last, seq)
      {
      }
    };
    

    template<typename OutIter>
    inline SwigPyIterator*
    make_output_value_iterator(const OutIter& current, const OutIter& begin, const OutIter& end, PyObject *seq = 0)
    {
      return new SwigPyMapValueITerator_T<OutIter>(current, begin, end, seq);
    }
  }
}

%define %swig_sequence_unordered_map_iterator(Sequence...)
#if defined(SWIG_EXPORT_ITERATOR_METHODS)
  class iterator;
  class const_iterator;

  %typemap(out,noblock=1,fragment="SwigPySequence_Cont")
    iterator, const_iterator {
    $result = SWIG_NewPointerObj(swig::make_output_forward_iterator(%static_cast($1,const $type &)),
                                 swig::SwigPyIterator::descriptor(),SWIG_POINTER_OWN);
  }
  %typemap(out,noblock=1,fragment="SwigPySequence_Cont")
    std::pair<iterator, iterator>, std::pair<const_iterator, const_iterator> {
    $result = PyTuple_New(2);
    PyTuple_SetItem($result,0,SWIG_NewPointerObj(swig::make_output_forward_iterator(%static_cast($1,const $type &).first),
                                                 swig::SwigPyIterator::descriptor(),SWIG_POINTER_OWN));
    PyTuple_SetItem($result,1,SWIG_NewPointerObj(swig::make_output_forward_iterator(%static_cast($1,const $type &).second),
                                                 swig::SwigPyIterator::descriptor(),SWIG_POINTER_OWN));    
  }

  %fragment("SwigPyPairBoolOutputIterator","header",fragment=SWIG_From_frag(bool),fragment="SwigPySequence_Cont") {}

  %typemap(out,noblock=1,fragment="SwigPyPairBoolOutputIterator")
    std::pair<iterator, bool>, std::pair<const_iterator, bool> {
    $result = PyTuple_New(2);
    PyTuple_SetItem($result,0,SWIG_NewPointerObj(swig::make_output_forward_iterator(%static_cast($1,const $type &).first),
                                               swig::SwigPyIterator::descriptor(),SWIG_POINTER_OWN));    
    PyTuple_SetItem($result,1,SWIG_From(bool)(%static_cast($1,const $type &).second));
  }

  %typemap(in,noblock=1,fragment="SwigPySequence_Cont")
    iterator(swig::SwigPyIterator *iter = 0, int res),
    const_iterator(swig::SwigPyIterator *iter = 0, int res) {
    res = SWIG_ConvertPtr($input, %as_voidptrptr(&iter), swig::SwigPyIterator::descriptor(), 0);
    if (!SWIG_IsOK(res) || !iter) {
      %argument_fail(SWIG_TypeError, "$type", $symname, $argnum);
    } else {
      swig::SwigPyIterator_T<$type > *iter_t = dynamic_cast<swig::SwigPyIterator_T<$type > *>(iter);
      if (iter_t) {
        $1 = iter_t->get_current();
      } else {
        %argument_fail(SWIG_TypeError, "$type", $symname, $argnum);
      }
    }
  }

  %typecheck(%checkcode(ITERATOR),noblock=1,fragment="SwigPySequence_Cont")
    iterator, const_iterator {
    swig::SwigPyIterator *iter = 0;
    int res = SWIG_ConvertPtr($input, %as_voidptrptr(&iter), swig::SwigPyIterator::descriptor(), 0);
    $1 = (SWIG_IsOK(res) && iter && (dynamic_cast<swig::SwigPyIterator_T<$type > *>(iter) != 0));
  }

  %fragment("SwigPySequence_Cont");

  %newobject iterator(PyObject **PYTHON_SELF);
  %extend  {
    swig::SwigPyIterator* iterator(PyObject **PYTHON_SELF) {
      return swig::make_output_forward_iterator(self->begin(), self->begin(), self->end(), *PYTHON_SELF);
    }

#if defined(SWIGPYTHON_BUILTIN)
  %feature("python:slot", "tp_iter", functype="getiterfunc") iterator;
#else
  %pythoncode {def __iter__(self):
    return self.iterator()}
#endif
  }

#endif //SWIG_EXPORT_ITERATOR_METHODS
%enddef

%define %swig_unordered_map_common(Map...)
  %swig_sequence_unordered_map_iterator(Map);
  %swig_container_methods(Map)

  %extend {
    mapped_type __getitem__(const key_type& key) const throw (std::out_of_range) {
      Map::const_iterator i = self->find(key);
      if (i != self->end())
        return i->second;
      else
        throw std::out_of_range("key not found");
    }
    
    void __delitem__(const key_type& key) throw (std::out_of_range) {
      Map::iterator i = self->find(key);
      if (i != self->end())
        self->erase(i);
      else
        throw std::out_of_range("key not found");
    }
    
    bool has_key(const key_type& key) const {
      Map::const_iterator i = self->find(key);
      return i != self->end();
    }
    
    PyObject* keys() {
      Map::size_type size = self->size();
      int pysize = (size <= (Map::size_type) INT_MAX) ? (int) size : -1;
      if (pysize < 0) {
        SWIG_PYTHON_THREAD_BEGIN_BLOCK;
        PyErr_SetString(PyExc_OverflowError,
                        "unordered_map size not valid in python");
        SWIG_PYTHON_THREAD_END_BLOCK;
        return NULL;
      }
      PyObject* keyList = PyList_New(pysize);
      Map::const_iterator i = self->begin();
      for (int j = 0; j < pysize; ++i, ++j) {
        PyList_SET_ITEM(keyList, j, swig::from(i->first));
      }
      return keyList;
    }
    
    PyObject* values() {
      Map::size_type size = self->size();
      int pysize = (size <= (Map::size_type) INT_MAX) ? (int) size : -1;
      if (pysize < 0) {
        SWIG_PYTHON_THREAD_BEGIN_BLOCK;
        PyErr_SetString(PyExc_OverflowError,
                        "unordered_map size not valid in python");
        SWIG_PYTHON_THREAD_END_BLOCK;
        return NULL;
      }
      PyObject* valList = PyList_New(pysize);
      Map::const_iterator i = self->begin();
      for (int j = 0; j < pysize; ++i, ++j) {
        PyList_SET_ITEM(valList, j, swig::from(i->second));
      }
      return valList;
    }
    
    PyObject* items() {
      Map::size_type size = self->size();
      int pysize = (size <= (Map::size_type) INT_MAX) ? (int) size : -1;
      if (pysize < 0) {
        SWIG_PYTHON_THREAD_BEGIN_BLOCK;
        PyErr_SetString(PyExc_OverflowError,
                        "unordered_map size not valid in python");
        SWIG_PYTHON_THREAD_END_BLOCK;
        return NULL;
      }    
      PyObject* itemList = PyList_New(pysize);
      Map::const_iterator i = self->begin();
      for (int j = 0; j < pysize; ++i, ++j) {
        PyList_SET_ITEM(itemList, j, swig::from(*i));
      }
      return itemList;
    }
    
    // Python 2.2 methods
    bool __contains__(const key_type& key) {
      return self->find(key) != self->end();
    }

    %newobject key_iterator(PyObject **PYTHON_SELF);
    swig::SwigPyIterator* key_iterator(PyObject **PYTHON_SELF) {
      return swig::make_output_key_iterator(self->begin(), self->begin(), self->end(), *PYTHON_SELF);
    }

    %newobject value_iterator(PyObject **PYTHON_SELF);
    swig::SwigPyIterator* value_iterator(PyObject **PYTHON_SELF) {
      return swig::make_output_value_iterator(self->begin(), self->begin(), self->end(), *PYTHON_SELF);
    }

    %pythoncode {def __iter__(self):
    return self.key_iterator()}    
    %pythoncode {def iterkeys(self):
    return self.key_iterator()}
    %pythoncode {def itervalues(self):
    return self.value_iterator()}
    %pythoncode {def iteritems(self):
    return self.iterator()}
  }
%enddef

%define %swig_unordered_map_methods(Map...)
  %swig_unordered_map_common(Map)
  %extend {
    void __setitem__(const key_type& key, const mapped_type& x) throw (std::out_of_range) {
      (*self)[key] = x;
    }
  }
%enddef


%include "unordered_map.i"