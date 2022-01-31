package org.firstinspires.ftc.robotcontroller.internal;

import android.content.Context;
import android.view.Menu;

import com.qualcomm.robotcore.util.ClassUtil;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.ftccommon.external.OnCreateMenu;
import org.firstinspires.ftc.ftccommon.external.OnDestroy;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.robotcore.internal.opmode.ClassFilter;
import org.firstinspires.ftc.robotcore.internal.opmode.OnBotJavaDeterminer;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class CustomAnnotatedHooksClassFilter implements ClassFilter {
  private static final String TAG = "CustomAnnotatedHooksClassFilter";
  private static final Predicate<Method> isOnBotJava =
      method -> OnBotJavaDeterminer.isOnBotJava(method.getDeclaringClass());
  private static final Predicate<Method> isExternalLibraries =
      method -> OnBotJavaDeterminer.isExternalLibraries(method.getDeclaringClass());

  private final Set<Method> onCreateMethods = new HashSet<>();
  private final Set<Method> onCreateEventLoopMethods = new HashSet<>();
  private final Set<Method> onCreateMenuMethods = new HashSet<>();
  private final Set<Method> onDestroyMethods = new HashSet<>();
  private final Set<Method> webHandlerRegistrarMethods = new HashSet<>();
  private final List<Set<Method>> allSets = new ArrayList<>();

  // Singleton

  private CustomAnnotatedHooksClassFilter() {
    allSets.add(onCreateMethods);
    allSets.add(onCreateEventLoopMethods);
    allSets.add(onCreateMenuMethods);
    allSets.add(onDestroyMethods);
    allSets.add(webHandlerRegistrarMethods);
  }

  public static CustomAnnotatedHooksClassFilter getInstance() {
    return CustomAnnotatedHooksClassFilter.InstanceHolder.theInstance;
  }

  @Override
  public void filterAllClassesStart() {
    clear();
  }

  // ClassFilter

  @Override
  public void filterOnBotJavaClassesStart() {
    removeOldMethods(isOnBotJava);
  }

  @Override
  public void filterExternalLibrariesClassesStart() {
    removeOldMethods(isExternalLibraries);
  }

  @Override
  public void filterClass(Class clazz) {
    exploreClass(clazz);
  }

  @Override
  public void filterOnBotJavaClass(Class clazz) {
    exploreClass(clazz);
  }

  @Override
  public void filterExternalLibrariesClass(Class clazz) {
    exploreClass(clazz);
  }

  @Override
  public void filterAllClassesComplete() {}

  @Override
  public void filterOnBotJavaClassesComplete() {}

  @Override
  public void filterExternalLibrariesClassesComplete() {}

  public void callOnCreateMethods(Context context) {
    for (Method method : onCreateMethods) {
      try {
        method.invoke(null, context);
      } catch (Exception e) {
        RobotLog.e(
            TAG,
            e,
            "failure while calling OnCreate annotated method "
                + method.getDeclaringClass()
                + "."
                + method.getName());
      }
    }
  }

  // public

  public void callOnCreateEventLoopMethods(Context context, CustomEventLoop ftcEventLoop) {
    for (Method method : onCreateEventLoopMethods) {
      try {
        method.invoke(null, context, ftcEventLoop);
      } catch (Exception e) {
        RobotLog.e(
            TAG,
            e,
            "failure while calling OnCreateEventLoop annotated method "
                + method.getDeclaringClass()
                + "."
                + method.getName());
      }
    }
  }

  public void callOnCreateMenuMethods(Context context, Menu menu) {
    for (Method method : onCreateMenuMethods) {
      try {
        method.invoke(null, context, menu);
      } catch (Exception e) {
        RobotLog.e(
            TAG,
            e,
            "failure while calling OnCreateMenu annotated method "
                + method.getDeclaringClass()
                + "."
                + method.getName());
      }
    }
  }

  public void callOnDestroyMethods(Context context) {
    for (Method method : onDestroyMethods) {
      try {
        method.invoke(null, context);
      } catch (Exception e) {
        RobotLog.e(
            TAG,
            e,
            "failure while calling OnDestory annotated method "
                + method.getDeclaringClass()
                + "."
                + method.getName());
      }
    }
  }

  public void callWebHandlerRegistrarMethods(Context context, WebHandlerManager webHandlerManager) {
    for (Method method : webHandlerRegistrarMethods) {
      try {
        method.invoke(null, context, webHandlerManager);
      } catch (Exception e) {
        RobotLog.e(
            TAG,
            e,
            "failure while calling WebHandlerRegistrar annotated method "
                + method.getDeclaringClass()
                + "."
                + method.getName());
      }
    }
  }

  private void clear() {
    for (Set<Method> set : allSets) {
      set.clear();
    }
  }

  // private

  private void removeOldMethods(Predicate<Method> predicate) {
    for (Set<Method> set : allSets) {
      set.removeIf(predicate::test);
    }
  }

  private void exploreClass(Class<?> clazz) {
    for (Method method : ClassUtil.getLocalDeclaredMethods(clazz)) {
      int modifiers = method.getModifiers();
      int requiredModifiers = Modifier.PUBLIC | Modifier.STATIC;
      if ((modifiers & requiredModifiers) != requiredModifiers) {
        continue;
      }
      int prohibitedModifiers = Modifier.ABSTRACT;
      if ((method.getModifiers() & prohibitedModifiers) != 0) {
        continue;
      }

      if (method.isAnnotationPresent(OnCreate.class) && method.getParameterTypes().length == 1) {
        onCreateMethods.add(method);
      }
      if (method.isAnnotationPresent(OnCreateEventLoop.class)
          && method.getParameterTypes().length == 2) {
        onCreateEventLoopMethods.add(method);
      }
      if (method.isAnnotationPresent(OnCreateMenu.class)
          && method.getParameterTypes().length == 2) {
        onCreateMenuMethods.add(method);
      }
      if (method.isAnnotationPresent(OnDestroy.class) && method.getParameterTypes().length == 1) {
        onDestroyMethods.add(method);
      }
      if (method.isAnnotationPresent(WebHandlerRegistrar.class)
          && method.getParameterTypes().length == 2) {
        webHandlerRegistrarMethods.add(method);
      }
    }
  }

  private static class InstanceHolder {
    public static CustomAnnotatedHooksClassFilter theInstance =
        new CustomAnnotatedHooksClassFilter();
  }
}
