/**
 * Sends an annotation.
 * @param annotation The annotation.
 */
option(Annotation, (const std::string&) annotation)
{
  initial_state(send)
  {
    transition
    {
      if(state_time)
        goto waitForNewAnnotation;
    }
    action
    {
      ANNOTATION("Behavior", annotation);
      lastAnnotationSend = annotation;
    }
  }

  target_state(waitForNewAnnotation)
  {
    transition
    {
      if(annotation != lastAnnotationSend)
        goto send;
    }
  }
}

std::string lastAnnotationSend;
