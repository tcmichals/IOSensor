


#pragma once
#include <cstdint>
#include <array>


template<typename T, size_t SIZE=10>
class ringBuffer
{
    
        
protected:
    
    std::array <T, SIZE> m_array;
    size_t m_head;
    size_t m_tail;
    size_t m_size;
    
protected:
    size_t modulo_inc (const  size_t val, const size_t modulus);
    size_t modulo_dec (const size_t val, const size_t modulus);
    
public:
        ringBuffer();
        virtual ~ringBuffer();
        
        bool push(T  item);
        bool pop(T & item);
        size_t size();
        bool isEmpty();
        bool isFull();
        size_t numItems();

};
template<typename T, size_t SIZE>
inline
ringBuffer<T, SIZE>::ringBuffer():m_head(0),
                                        m_tail(0),
                                        m_size(0)
                                        
{
    
}

template<typename T, size_t SIZE>
inline
ringBuffer<T, SIZE>::~ringBuffer()
{
}


template<typename T, size_t SIZE>
inline
size_t ringBuffer<T, SIZE>::modulo_inc (const  size_t val, const size_t modulus)
{
        size_t _val = val + 1;
        if (_val >= modulus)
        {
        _val  = 0;
        }
        return (_val);
}

template<typename T, size_t SIZE>
inline
size_t ringBuffer<T, SIZE>::modulo_dec (const size_t val, const size_t modulus)
{
        size_t _val = (0==val) ? (modulus - 1) : (val - 1);
        return (_val);
}

   
template<typename T, size_t SIZE>
inline   
bool ringBuffer<T, SIZE>::push(T item)
{

  if ( (m_size +1) <= m_array.size())
  {
      m_array[m_head] = item;
      m_head = modulo_inc ( m_head, SIZE);
      m_size++;
    
    return true;

  }
  
  return false;
}

template<typename T, size_t SIZE>
inline   
bool ringBuffer<T, SIZE>::pop(T & item)
{
    if (m_size==0)
        return false;
    
    item = m_array[m_tail];
    m_tail = modulo_inc(m_tail, SIZE);
    m_size--;
    
    return true;
}

template<typename T, size_t SIZE>
inline  
size_t ringBuffer<T, SIZE>::size()
{
    return m_size;
}

template<typename T, size_t SIZE>
inline  
bool ringBuffer<T, SIZE>::isEmpty()
{
    if (m_size == 0)
        return true;
    
    return false;
}

template<typename T, size_t SIZE>
inline  
bool ringBuffer<T, SIZE>::isFull()
{
    if (m_size == m_array.size())
        return true;
    
    return false;
}


template<typename T, size_t SIZE>
inline  
size_t ringBuffer<T, SIZE>::numItems()
{
    return m_array.size();
}

//eof

